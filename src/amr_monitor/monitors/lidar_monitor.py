# -*- coding: utf-8 -*-
"""LiDAR Monitor
src/amr_monitor/monitors/lidar_monitor.py
"""

# 导入 Qt 相关库
from PySide6.QtCore import QObject

# 导入监视器 UI
from amr_monitor.ui.monitors.lidar_monitor_ui import Ui_LidarMonitor


class LidarMonitor(QObject, Ui_LidarMonitor):
    """LiDAR Monitor"""

    def __init__(self, parent=None):
        """初始化"""
        super().__init__(parent)
        self.setupUi(parent)

        # 初始化信号和槽
        self.connect_signals()

    def connect_signals(self):
        """连接信号"""

        pass
