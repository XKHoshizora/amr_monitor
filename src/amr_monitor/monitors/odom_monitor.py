# -*- coding: utf-8 -*-
"""Odometry Monitor
src/amr_monitor/monitors/odom_monitor.py
"""

# 导入 Qt 相关库
from PySide6.QtCore import QObject

# 导入监视器 UI
from amr_monitor.ui.monitors.odom_monitor_ui import Ui_OdomMonitor


class OdomMonitor(QObject, Ui_OdomMonitor):
    """Odometry Monitor"""

    def __init__(self, parent=None):
        """初始化"""
        super().__init__(parent)
        self.setupUi(parent)

        # 初始化信号和槽
        self.connect_signals()

    def connect_signals(self):
        """连接 OdomMonitor 的信号和槽函数"""
        self.refreshButton.clicked.connect(self.on_refresh_button_clicked)
        self.clearTrajectoryButton.clicked.connect(self.on_clear_trajectory_button_clicked)

    def on_refresh_button_clicked(self):
        print("刷新按钮点击")

    def on_clear_trajectory_button_clicked(self):
        print("清除轨迹按钮点击")
