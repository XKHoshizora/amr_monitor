# -*- coding: utf-8 -*-
"""AMR Monitor
src/amr_monitor/monitor.py
"""

# 导入标准库
import sys
from pathlib import Path
import io

# 改变默认编码来支持中文输出
sys.stdout = io.TextIOWrapper(sys.stdout.buffer, encoding='utf-8')
sys.stdout.reconfigure(line_buffering=True)  # 设置为行缓冲模式

# 导入 Qt 相关库
from PySide6.QtWidgets import QApplication, QMainWindow, QWidget

# 获取项目根目录路径
project_root = Path(__file__).resolve().parents[2]

# 将 src 目录添加到 sys.path 中
src_path = project_root / "src"
sys.path.append(str(src_path))

# 导入主窗口 UI
from amr_monitor.ui.main_window_ui import Ui_MainWindow
# 导入监视器
from amr_monitor.monitors.imu_monitor import IMUMonitor
from amr_monitor.monitors.odom_monitor import OdomMonitor
from amr_monitor.monitors.lidar_monitor import LidarMonitor
from amr_monitor.monitors.cmd_vel_monitor import CmdVelMonitor


class MainWindow(QMainWindow, Ui_MainWindow):
    def __init__(self):
        super().__init__()

        # 主窗口初始化
        self.uiMainWindow = Ui_MainWindow()
        self.uiMainWindow.setupUi(self)

        # 示例：将 OdomMonitor UI 界面嵌入到 odomTab 中
        # self.add_monitor_to_tab(Ui_OdomMonitor, self.uiMainWindow.odomLayout)

        # 将 OdomMonitor 嵌入到 odomTab 中
        self.add_monitor_widget(OdomMonitor, self.uiMainWindow.odomLayout)
        # 将 ImuMonitor 嵌入到 imuTab 中
        self.add_monitor_widget(IMUMonitor, self.uiMainWindow.imuLayout)
        # 将 LidarMonitor 嵌入到 lidarTab 中
        self.add_monitor_widget(LidarMonitor, self.uiMainWindow.lidarLayout)
        # 将 CmdVelMonitor 嵌入到 cmdVelTab 中
        self.add_monitor_widget(CmdVelMonitor, self.uiMainWindow.cmdVelLayout)

        # 初始化信号和槽
        self.connect_signals()

    def add_monitor_to_tab(self, ui_class, layout):
        """
        将指定的监视器界面添加到给定的布局中。

        参数:
        - ui_class: 要添加的监视器界面的 UI 类（例如 Ui_OdomMonitor）。
        - layout: 添加到的目标布局（例如 self.uiMainWindow.odomLayout）。
        """
        # 创建并初始化监视器 UI 实例和容器
        monitor_ui = ui_class()
        monitor_widget = QWidget()
        monitor_ui.setupUi(monitor_widget)

        # 将监视器界面添加到指定的布局中
        layout.addWidget(monitor_widget)

    def add_monitor_widget(self, monitor_class, layout):
        """
        将指定的监视器类实例化并添加到主窗口中的指定布局中。

        参数:
        - monitor_class: 要实例化的监视器类，例如 OdomMonitor。
        - layout: 要添加到的目标布局，例如 self.uiMainWindow.odomLayout。
        """
        # 创建监视器小部件和实例
        monitor_widget = QWidget()
        monitor_instance = monitor_class(monitor_widget)

        # 将监视器小部件添加到指定的布局
        layout.addWidget(monitor_widget)

    
    def connect_signals(self):
        """连接信号"""

        pass


if __name__ == "__main__":
    app = QApplication(sys.argv)

    window = MainWindow()
    window.show()

    app.exec()
