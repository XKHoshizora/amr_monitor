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
        self.resize(
            config['ui']['window']['width'],
            config['ui']['window']['height']
        )

        self.setup_ui()
        self.setup_menu()

    def setup_ui(self):
        # 创建中心部件和布局
        central_widget = QWidget()
        self.setCentralWidget(central_widget)
        layout = QHBoxLayout(central_widget)

        # 创建左右分割窗口
        splitter = QSplitter(Qt.Horizontal)
        layout.addWidget(splitter)

        # 左侧放置传感器数据显示
        left_widget = QWidget()
        left_layout = QVBoxLayout(left_widget)

        # 传感器数据图表
        self.plot_widget = PlotWidget()
        left_layout.addWidget(self.plot_widget)

        # 添加数据记录控制
        record_group = QGroupBox("数据记录")
        record_layout = QHBoxLayout()
        self.record_button = QPushButton("开始记录")
        self.record_button.setCheckable(True)
        self.record_button.toggled.connect(self.toggle_recording)
        record_layout.addWidget(self.record_button)
        record_group.setLayout(record_layout)
        left_layout.addWidget(record_group)

        splitter.addWidget(left_widget)

        # 右侧放置参数显示和控制
        self.parameter_widget = ParameterWidget()
        splitter.addWidget(self.parameter_widget)

        # 初始化数据记录器
        self.recording = False
        self.data_buffer = []

        # 设置定时器更新UI
        self.timer = QTimer()
        self.timer.timeout.connect(self.update_data)
        self.timer.start(100)  # 10Hz更新率

    def setup_menu(self):
        menubar = self.menuBar()

        # 文件菜单
        file_menu = menubar.addMenu('文件')

        export_action = QAction('导出数据', self)
        export_action.triggered.connect(self.export_data)
        file_menu.addAction(export_action)

        playback_action = QAction('数据回放', self)
        playback_action.triggered.connect(self.show_playback)
        file_menu.addAction(playback_action)

        # 视图菜单
        view_menu = menubar.addMenu('视图')

        theme_menu = view_menu.addMenu('主题')
        dark_action = QAction('暗色主题', self)
        light_action = QAction('亮色主题', self)
        dark_action.triggered.connect(
            lambda: self.theme_manager.apply_dark_theme(QApplication.instance()))
        light_action.triggered.connect(
            lambda: self.theme_manager.apply_light_theme(QApplication.instance()))
        theme_menu.addAction(dark_action)
        theme_menu.addAction(light_action)

        # 工具菜单
        tools_menu = menubar.addMenu('工具')

        analysis_action = QAction('数据分析', self)
        analysis_action.triggered.connect(self.show_analysis)
        tools_menu.addAction(analysis_action)

    def init_managers(self):
        self.theme_manager = ThemeManager()
        self.config_manager = ConfigManager()

    def toggle_recording(self, checked):
        if checked:
            timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
            self.filename = os.path.join(self.data_dir, f"amr_data_{timestamp}.csv")
            self.record_button.setText("停止记录")
            self.recording = True
        else:
            self.save_recorded_data()
            self.record_button.setText("开始记录")
            self.recording = False
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
        if not hasattr(self, 'playback_widget'):
            try:
                self.playback_widget = PlaybackWidget(
                    data_dir=self.data_dir,
                    parent=self
                )
                self.playback_widget.data_updated.connect(
                    self.plot_widget.update_from_playback)
            except Exception as e:
                rospy.logerr(f"创建回放窗口失败: {str(e)}")
                QMessageBox.critical(self, "错误", f"创建回放窗口失败: {str(e)}")
                return
        self.playback_widget.show()

    def show_analysis(self):
        """显示数据分析窗口"""
        if not hasattr(self, 'analysis_widget'):
            try:
                self.analysis_widget = AnalysisWidget(
                    data_dir=self.data_dir,
                    parent=self
                )
            except Exception as e:
                rospy.logerr(f"创建分析窗口失败: {str(e)}")
                QMessageBox.critical(self, "错误", f"创建分析窗口失败: {str(e)}")
                return
        self.analysis_widget.show()

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
            # 保存窗口大小到配置
            config = self.config_manager.get_config()
            config['ui']['window']['width'] = self.width()
            config['ui']['window']['height'] = self.height()
            self.config_manager.save_config(config)
            event.accept()
