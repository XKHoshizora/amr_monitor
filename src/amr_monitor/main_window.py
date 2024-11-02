# src/amr_monitor/main_window.py
"""主窗口实现"""
import sys
import time
import threading
from typing import Optional

import rospy
from PySide6.QtWidgets import (QMainWindow, QWidget, QVBoxLayout, QTabWidget,
                               QMessageBox, QFileDialog, QApplication, QLabel)
from PySide6.QtCore import Qt, QSettings, QTimer, Signal

from amr_monitor.core.config import ConfigManager
from amr_monitor.core.event_bus import EventBus, Event
from amr_monitor.core.logger import Logger
from amr_monitor.core.error_handler import handle_error, ErrorType, ErrorSeverity
from amr_monitor.core.monitor_manager import monitor_manager
from amr_monitor.core.topic_manager import topic_manager
from amr_monitor.utils.storage import StorageManager
from amr_monitor.ui.themes.theme_manager import ThemeManager

from amr_monitor.monitors import (
    IMUMonitor,
    OdomMonitor,
    LidarMonitor,
    CmdVelMonitor,
    BaseMonitor
)

logger = Logger.get_logger(__name__)


class MainWindow(QMainWindow):
    """AMR监控器主窗口"""

    def __init__(self, config_manager: ConfigManager,
                 event_bus: EventBus, storage_manager: StorageManager):
        super().__init__()
        # 核心组件
        self.config_manager = config_manager
        self.event_bus = event_bus
        self.storage_manager = storage_manager
        self.theme_manager = ThemeManager()

        # UI初始化
        self.setup_ui()
        self.setup_monitors()
        self.setup_events()

        # 加载主题和布局
        self.load_theme()
        self.load_layout()

        # 设置窗口属性
        self.setWindowTitle("AMR Monitor")
        self.resize(1200, 800)

        # 设置定时同步
        self.sync_timer = QTimer(self)
        self.sync_timer.timeout.connect(self.sync_monitors)
        self.sync_timer.start(1000)  # 每秒同步一次

    def setup_ui(self):
        """设置UI"""
        # 创建中央部件
        self.central_widget = QWidget()
        self.setCentralWidget(self.central_widget)

        # 主布局
        self.main_layout = QVBoxLayout(self.central_widget)

        # 标签页
        self.tab_widget = QTabWidget()
        self.main_layout.addWidget(self.tab_widget)

        # 连接标签页切换信号
        self.tab_widget.currentChanged.connect(self._handle_tab_changed)

        # 创建菜单
        self.create_menu_bar()

        # 创建状态栏
        self.statusBar().showMessage("Ready")

    def create_menu_bar(self):
        """创建菜单栏"""
        menubar = self.menuBar()

        # 文件菜单
        file_menu = menubar.addMenu("File")
        file_menu.addAction("Save Layout", self.save_layout)
        file_menu.addAction("Load Layout", self.load_layout)
        file_menu.addAction("Export Data", self.export_all_data)
        file_menu.addSeparator()
        file_menu.addAction("Exit", self.close)

        # 视图菜单
        view_menu = menubar.addMenu("View")
        view_menu.addAction("Reset Layout", self.reset_layout)
        view_menu.addAction("Refresh All", self.refresh_all_topics)

        # 主题子菜单
        theme_menu = view_menu.addMenu("Theme")
        theme_actions = {}
        for theme_name in self.theme_manager.get_available_themes():
            action = theme_menu.addAction(theme_name)
            action.setCheckable(True)
            theme_actions[theme_name] = action
            action.triggered.connect(
                lambda checked, name=theme_name: self.apply_theme(name))

        # 工具菜单
        tools_menu = menubar.addMenu("Tools")
        tools_menu.addAction("Clear All Data", self.clear_all_data)
        tools_menu.addAction("Monitor Status", self.show_monitor_status)

        # 帮助菜单
        help_menu = menubar.addMenu("Help")
        help_menu.addAction("About", self.show_about)

    def setup_monitors(self):
        """设置监控器"""
        monitor_classes = {
            'IMU': IMUMonitor,
            'Odometry': OdomMonitor,
            'LiDAR': LidarMonitor,
            'CMD_VEL': CmdVelMonitor
        }

        for name, monitor_class in monitor_classes.items():
            try:
                config = self.config_manager.get_monitor_config(name.lower())
                monitor = monitor_class(config, self.event_bus)
                self.add_monitor(name, monitor)
            except Exception as e:
                error_msg = f"Failed to create {name} monitor: {str(e)}"
                handle_error(ErrorType.SYSTEM, ErrorSeverity.ERROR,
                             error_msg, "MainWindow")

    def add_monitor(self, name: str, monitor: BaseMonitor):
        """添加监控器"""
        if monitor_manager.register_monitor(name, monitor):
            self.tab_widget.addTab(monitor, name)
            monitor_manager.monitor_status_changed.connect(
                lambda mon, status: self._update_monitor_status(mon, status))
            monitor_manager.monitor_error.connect(
                lambda mon, error: self._handle_monitor_error(mon, error))

    def setup_events(self):
        """设置事件处理"""
        self.event_bus.subscribe('monitor_data', self._handle_monitor_data)
        self.event_bus.subscribe('error', self._handle_error)

    def _handle_monitor_data(self, event: Event):
        """处理监控数据"""
        if self.storage_manager:
            try:
                self.storage_manager.store_data(
                    event.source,
                    'monitor_data',
                    event.data
                )
            except Exception as e:
                logger.error(f"Failed to store data: {e}")

    def _handle_error(self, event: Event):
        """处理错误事件"""
        handle_error(
            ErrorType.SYSTEM,
            ErrorSeverity.ERROR,
            str(event.data),
            event.source
        )
        self.statusBar().showMessage(f"Error: {event.data}", 5000)

    def _handle_tab_changed(self, index: int):
        if index >= 0:
            current_name = self.tab_widget.tabText(index)
            # 先停用所有监控器
            active_monitors = monitor_manager.get_active_monitors()
            for name in active_monitors:
                if name != current_name:
                    monitor_manager.deactivate_monitor(name)
            # 再激活当前监控器
            monitor_manager.activate_monitor(current_name)

    def _update_monitor_status(self, monitor_name: str, status: str):
        """更新监控器状态显示"""
        index = self._get_tab_index(monitor_name)
        if index >= 0:
            current_text = self.tab_widget.tabText(index)
            if status == "error":
                self.tab_widget.setTabText(index, f"{current_text} ⚠")
            else:
                self.tab_widget.setTabText(index, monitor_name)

    def _handle_monitor_error(self, monitor_name: str, error: str):
        """处理监控器错误"""
        handle_error(
            ErrorType.SYSTEM,
            ErrorSeverity.ERROR,
            error,
            monitor_name
        )
        self.statusBar().showMessage(f"Error in {monitor_name}: {error}", 5000)

    def _get_tab_index(self, name: str) -> int:
        """获取标签页索引"""
        for i in range(self.tab_widget.count()):
            if self.tab_widget.tabText(i).startswith(name):
                return i
        return -1

    def sync_monitors(self):
        """同步所有监控器"""
        monitor_manager.sync_monitors()

    def refresh_all_topics(self):
        """刷新所有话题"""
        for monitor in monitor_manager.monitors.values():
            if hasattr(monitor, '_on_refresh_clicked'):
                monitor._on_refresh_clicked()

    def show_monitor_status(self):
        """显示监控器状态"""
        status_text = "Monitor Status:\n\n"
        for name, status in monitor_manager.monitor_status.items():
            status_text += (f"{name}:\n"
                            f"  Status: {status.health_status}\n"
                            f"  Active: {status.is_active}\n"
                            f"  Errors: {status.error_count}\n"
                            f"  Last Update: {time.ctime(status.last_update)}\n\n")

        QMessageBox.information(self, "Monitor Status", status_text)

    def show_about(self):
        """显示关于对话框"""
        QMessageBox.about(
            self,
            "About AMR Monitor",
            "AMR Monitor v1.0.0\n\n"
            "A monitoring tool for AMR robots\n\n"
            "Created by Hoshizora"
        )

    def load_theme(self):
        """加载主题"""
        theme = self.config_manager.get_config('ui', {}).get('theme', 'dark')
        self.apply_theme(theme)

    def apply_theme(self, theme_name: str):
        """应用主题"""
        if self.theme_manager.apply_theme(self, theme_name):
            self.config_manager.set_config(['ui', 'theme'], theme_name)

    def save_layout(self):
        """保存布局"""
        try:
            settings = QSettings('AMR_Monitor', 'Layout')
            settings.setValue('geometry', self.saveGeometry())
            settings.setValue('state', self.saveState())
            settings.setValue('tab_index', self.tab_widget.currentIndex())
            self.statusBar().showMessage("Layout saved", 3000)
        except Exception as e:
            logger.error(f"Failed to save layout: {e}")

    def load_layout(self):
        """加载布局"""
        try:
            settings = QSettings('AMR_Monitor', 'Layout')
            geometry = settings.value('geometry')
            state = settings.value('state')
            tab_index = settings.value('tab_index', 0, type=int)

            if geometry:
                self.restoreGeometry(geometry)
            if state:
                self.restoreState(state)
            self.tab_widget.setCurrentIndex(tab_index)
        except Exception as e:
            logger.error(f"Failed to load layout: {e}")

    def reset_layout(self):
        """重置布局"""
        reply = QMessageBox.question(
            self,
            "Reset Layout",
            "Are you sure you want to reset the layout?",
            QMessageBox.Yes | QMessageBox.No
        )

        if reply == QMessageBox.Yes:
            self.resize(1200, 800)
            self.tab_widget.setCurrentIndex(0)
            for monitor in monitor_manager.monitors.values():
                if hasattr(monitor, 'reset_layout'):
                    monitor.reset_layout()

    def export_all_data(self):
        """导出所有数据"""
        if not self.storage_manager:
            QMessageBox.warning(self, "Warning", "Storage is not enabled")
            return

        try:
            directory = QFileDialog.getExistingDirectory(
                self, "Select Export Directory")

            if directory:
                for name in monitor_manager.monitors.keys():
                    filename = f"{directory}/{name}_{time.strftime('%Y%m%d_%H%M%S')}.csv"
                    if self.storage_manager.export_data(name, filename):
                        logger.info(f"Exported data for {name} to {filename}")
                    else:
                        logger.warning(f"No data to export for {name}")

                self.statusBar().showMessage("Data exported successfully", 3000)
        except Exception as e:
            error_msg = f"Failed to export data: {str(e)}"
            handle_error(ErrorType.SYSTEM, ErrorSeverity.ERROR,
                         error_msg, "MainWindow")

    def clear_all_data(self):
        """清空所有数据"""
        reply = QMessageBox.question(
            self, 'Confirm Clear',
            'Are you sure you want to clear all data?',
            QMessageBox.Yes | QMessageBox.No,
            QMessageBox.No
        )

        if reply == QMessageBox.Yes:
            try:
                if self.storage_manager:
                    self.storage_manager.clear_all()
                for monitor in monitor_manager.monitors.values():
                    monitor.clear_data()
                self.statusBar().showMessage("All data cleared", 3000)
            except Exception as e:
                error_msg = f"Failed to clear data: {str(e)}"
                handle_error(ErrorType.SYSTEM, ErrorSeverity.ERROR,
                             error_msg, "MainWindow")

    def closeEvent(self, event):
        """关闭事件处理"""
        reply = QMessageBox.question(
            self, 'Confirm Exit',
            'Are you sure you want to exit?',
            QMessageBox.Yes | QMessageBox.No,
            QMessageBox.No
        )

        if reply == QMessageBox.Yes:
            self.cleanup()
            event.accept()
        else:
            event.ignore()

    def cleanup(self):
        """清理资源"""
        try:
            # 停止同步定时器
            if hasattr(self, 'sync_timer'):
                self.sync_timer.stop()

            # 需要在停止定时器后等待定时器回调完成
            QApplication.processEvents()

            # 停止所有监控器
            for name in list(monitor_manager.monitors.keys()):
                try:
                    monitor_manager.deactivate_monitor(name)
                    monitor_manager.unregister_monitor(name)
                except Exception as e:
                    logger.error(f"Failed to cleanup monitor {name}: {e}")

            # 取消所有话题订阅
            topic_manager.cleanup()

            # 保存布局
            self.save_layout()

            # 关闭存储
            if self.storage_manager:
                try:
                    self.storage_manager.close()
                except Exception as e:
                    logger.error(f"Failed to close storage manager: {e}")

            # 清理事件总线
            self.event_bus.clear()

            # 断开所有信号连接
            self.disconnect_signals()

        except Exception as e:
            logger.error(f"Error during cleanup: {e}")

    def disconnect_signals(self):
        """断开所有信号连接"""
        try:
            # 断开标签页切换信号
            self.tab_widget.currentChanged.disconnect()

            # 断开监控器管理器信号
            monitor_manager.monitor_status_changed.disconnect()
            monitor_manager.monitor_error.disconnect()

            # 断开事件总线订阅
            self.event_bus.unsubscribe(
                'monitor_data', self._handle_monitor_data)
            self.event_bus.unsubscribe('error', self._handle_error)
        except Exception as e:
            logger.error(f"Error disconnecting signals: {e}")
