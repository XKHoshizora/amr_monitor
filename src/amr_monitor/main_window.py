import os
import sys
import rospy
import rospkg
from PyQt5.QtWidgets import *
from PyQt5.QtCore import *
from datetime import datetime
import pandas as pd
from .plot_widget import PlotWidget
from .parameter_widget import ParameterWidget
from .playback_widget import PlaybackWidget
from .theme_manager import ThemeManager
from .config_manager import ConfigManager
from .analysis_widget import AnalysisWidget


class MainWindow(QMainWindow):
    def __init__(self):
        super().__init__()
        # 初始化包路径
        rospack = rospkg.RosPack()
        self.package_path = rospack.get_path('amr_monitor')
        self.data_dir = os.path.join(self.package_path, 'data')

        # 确保数据目录存在
        os.makedirs(self.data_dir, exist_ok=True)

        # 初始化管理器
        self.init_managers()

        # 应用配置
        config = self.config_manager.get_config()
        self.setWindowTitle('AMR Monitor')

        # 设置窗口属性
        self.setMinimumSize(800, 600)  # 设置最小尺寸
        self.resize(
            config['ui']['window']['width'],
            config['ui']['window']['height']
        )

        # 设置窗口状态
        self.previous_size = None  # 用于存储最大化前的尺寸
        self.setup_ui()
        self.setup_menu()

    def setup_ui(self):
        # 创建中心部件和布局
        self.central_widget = QWidget()
        self.setCentralWidget(self.central_widget)
        self.main_layout = QHBoxLayout(self.central_widget)
        self.main_layout.setContentsMargins(5, 5, 5, 5)  # 设置边距
        self.main_layout.setSpacing(5)  # 设置组件间距

        # 创建左右分割窗口
        self.splitter = QSplitter(Qt.Horizontal)
        self.splitter.setHandleWidth(5)  # 设置分割条宽度
        self.main_layout.addWidget(self.splitter)

        # 左侧放置传感器数据显示
        left_widget = QWidget()
        left_layout = QVBoxLayout(left_widget)
        left_layout.setContentsMargins(0, 0, 0, 0)

        # 传感器数据图表
        self.plot_widget = PlotWidget()
        left_layout.addWidget(self.plot_widget, stretch=1)  # 添加拉伸因子

        # 添加数据记录控制
        record_group = QGroupBox("数据记录")
        record_layout = QHBoxLayout()
        record_layout.setContentsMargins(5, 5, 5, 5)
        self.record_button = QPushButton("开始记录")
        self.record_button.setCheckable(True)
        self.record_button.toggled.connect(self.toggle_recording)
        record_layout.addWidget(self.record_button)
        record_group.setLayout(record_layout)
        left_layout.addWidget(record_group)

        self.splitter.addWidget(left_widget)

        # 右侧放置参数显示和控制
        self.parameter_widget = ParameterWidget()
        self.splitter.addWidget(self.parameter_widget)

        # 设置分割比例
        self.splitter.setStretchFactor(0, 2)  # 左侧占比大
        self.splitter.setStretchFactor(1, 1)  # 右侧占比小

        # 初始化数据记录器
        self.recording = False
        self.data_buffer = []

        # 设置定时器更新UI
        self.timer = QTimer()
        self.timer.timeout.connect(self.update_data)
        self.timer.start(100)  # 10Hz更新率

    def changeEvent(self, event):
        """处理窗口状态改变事件"""
        if event.type() == QEvent.WindowStateChange:
            if self.windowState() & Qt.WindowMaximized:
                # 窗口最大化时保存之前的大小
                if not self.previous_size:
                    self.previous_size = self.size()
            else:
                # 从最大化恢复时使用保存的大小
                if self.previous_size:
                    self.resize(self.previous_size)
                    self.previous_size = None
        super().changeEvent(event)

    def setup_menu(self):
        self.menubar = self.menuBar()
        self.menubar.setNativeMenuBar(False)

        # 文件菜单
        self.file_menu = QMenu('文件(&F)', self)
        self.menubar.addMenu(self.file_menu)

        self.export_action = QAction('导出数据(&E)', self)
        self.export_action.triggered.connect(self.export_data)
        self.export_action.setShortcut('Ctrl+E')
        self.file_menu.addAction(self.export_action)

        self.playback_action = QAction('数据回放(&P)', self)
        self.playback_action.triggered.connect(self.show_playback)
        self.playback_action.setShortcut('Ctrl+P')
        self.file_menu.addAction(self.playback_action)

        # 视图菜单
        self.view_menu = QMenu('视图(&V)', self)
        self.menubar.addMenu(self.view_menu)

        # 创建数据视图子菜单并添加基本选项
        self.data_view_menu = QMenu('数据视图(&D)', self)
        self.view_menu.addMenu(self.data_view_menu)

        # 添加默认的数据视图选项
        self.view_actions = {}
        default_views = {
            'IMU数据': True,
            '里程计数据': True,
            '激光雷达数据': True
        }
        for title, initial_state in default_views.items():
            action = QAction(title, self)
            action.setCheckable(True)
            action.setChecked(initial_state)
            action.triggered.connect(lambda checked, t=title: self.toggle_tab_view(t, checked))
            self.data_view_menu.addAction(action)
            self.view_actions[title] = action

        # 主题菜单
        self.theme_menu = self.view_menu.addMenu('主题(&T)')
        self.dark_action = QAction('暗色主题(&D)', self)
        self.light_action = QAction('亮色主题(&L)', self)
        self.dark_action.triggered.connect(
            lambda: self.theme_manager.apply_dark_theme(QApplication.instance()))
        self.light_action.triggered.connect(
            lambda: self.theme_manager.apply_light_theme(QApplication.instance()))
        self.theme_menu.addAction(self.dark_action)
        self.theme_menu.addAction(self.light_action)

        # 工具菜单
        self.tools_menu = QMenu('工具(&T)', self)
        self.menubar.addMenu(self.tools_menu)

        self.analysis_action = QAction('数据分析(&A)', self)
        self.analysis_action.triggered.connect(self.show_analysis)
        self.analysis_action.setShortcut('Ctrl+A')
        self.tools_menu.addAction(self.analysis_action)

        # 存储已关闭的标签页
        self.closed_tabs = {}

    def add_to_view_menu(self, tab, title):
        """添加标签页到视图菜单"""
        # 存储标签页
        self.closed_tabs[title] = tab

        # 创建动作
        action = QAction(title, self)
        action.setCheckable(True)
        action.setChecked(False)
        action.triggered.connect(lambda checked: self.toggle_tab_view(title, checked))

        # 添加到数据视图菜单
        self.data_view_menu.addAction(action)

    def toggle_tab_view(self, title, checked):
        """切换标签页显示状态"""
        if hasattr(self.plot_widget, 'tab_widget'):
            tab_widget = self.plot_widget.tab_widget

            if checked:
                # 如果是选中状态，显示标签页
                if title in self.closed_tabs:
                    tab = self.closed_tabs[title]
                    tab_widget.addTab(tab, title)
                    del self.closed_tabs[title]
                # 如果标签页不在关闭列表中，可能需要重新创建
                elif title in self.plot_widget.original_tab_contents:
                    tab = self.plot_widget.original_tab_contents[title]
                    tab_widget.addTab(tab, title)
            else:
                # 如果是取消选中状态，隐藏标签页
                for i in range(tab_widget.count()):
                    if tab_widget.tabText(i) == title:
                        tab = tab_widget.widget(i)
                        self.closed_tabs[title] = tab
                        tab_widget.removeTab(i)
                        break

            # 确保菜单项状态与实际显示状态同步
            if title in self.view_actions:
                is_visible = any(tab_widget.tabText(i) == title
                            for i in range(tab_widget.count()))
                self.view_actions[title].setChecked(is_visible)

    def init_managers(self):
        self.theme_manager = ThemeManager()
        self.config_manager = ConfigManager()

    def toggle_recording(self, checked):
        if checked:
            timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
            self.filename = os.path.join(self.data_dir, f"amr_data_{timestamp}.csv")
            self.record_button.setText("停止记录")
            self.recording = True
            self.plot_widget.set_recording(True)  # 设置图表记录状态
        else:
            self.save_recorded_data()
            self.record_button.setText("开始记录")
            self.recording = False
            self.plot_widget.set_recording(False)  # 设置图表记录状态
            self.data_buffer = []

    def save_recorded_data(self):
        if self.data_buffer:
            try:
                df = pd.DataFrame(self.data_buffer)
                df.to_csv(self.filename, index=False)
                QMessageBox.information(
                    self,
                    "保存成功",
                    f"数据已保存至: {os.path.relpath(self.filename, self.package_path)}"
                )
            except Exception as e:
                QMessageBox.critical(
                    self,
                    "保存失败",
                    f"保存数据失败: {str(e)}"
                )

    def update_data(self):
        # 更新图表显示
        self.plot_widget.update_plots()

        # 如果正在记录，则保存数据
        if self.recording:
            data = self.plot_widget.get_current_data()
            self.data_buffer.append(data)

    def export_data(self):
        if not self.data_buffer:
            QMessageBox.warning(self, "警告", "没有可导出的数据")
            return

        filename, _ = QFileDialog.getSaveFileName(
            self,
            "导出数据",
            os.path.join(self.data_dir, "exported_data.csv"),
            "CSV Files (*.csv)"
        )
        if filename:
            try:
                df = pd.DataFrame(self.data_buffer)
                df.to_csv(filename, index=False)
                QMessageBox.information(
                    self,
                    "导出成功",
                    f"数据已导出至: {os.path.relpath(filename, self.package_path)}"
                )
            except Exception as e:
                QMessageBox.critical(
                    self,
                    "导出失败",
                    f"导出数据失败: {str(e)}"
                )

    def show_playback(self):
        """显示数据回放窗口"""
        try:
            # 总是创建新窗口
            self.playback_widget = PlaybackWidget(
                data_dir=self.data_dir,
                parent=None  # 设置为None使其成为独立窗口
            )
            self.playback_widget.data_updated.connect(
                self.plot_widget.update_from_playback)
            self.playback_widget.show()
            self.playback_widget.raise_()
            self.playback_widget.activateWindow()
        except Exception as e:
            rospy.logerr(f"显示回放窗口失败: {str(e)}")
            QMessageBox.critical(self, "错误", f"显示回放窗口失败: {str(e)}")

    def show_analysis(self):
        """显示数据分析窗口"""
        try:
            # 总是创建新窗口
            self.analysis_widget = AnalysisWidget(
                data_dir=self.data_dir,
                parent=None  # 设置为None使其成为独立窗口
            )
            self.analysis_widget.show()
            self.analysis_widget.raise_()
            self.analysis_widget.activateWindow()
        except Exception as e:
            rospy.logerr(f"显示分析窗口失败: {str(e)}")
            QMessageBox.critical(self, "错误", f"显示分析窗口失败: {str(e)}")

    def closeEvent(self, event):
        """处理关闭事件"""
        if self.recording:
            reply = QMessageBox.question(
                self, '确认退出',
                '正在记录数据，确定要退出吗？',
                QMessageBox.Yes | QMessageBox.No,
                QMessageBox.No
            )
            if reply == QMessageBox.Yes:
                self.toggle_recording(False)
                event.accept()
            else:
                event.ignore()
        else:
            # 关闭所有分离的窗口
            if hasattr(self.plot_widget, 'detached_windows'):
                for window in self.plot_widget.detached_windows.values():
                    window.close()
            event.accept()
