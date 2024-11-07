# -*- coding: utf-8 -*-
"""Cmd_Vel Monitor
src/amr_monitor/monitors/imu_monitor.py
"""

# 导入 Qt 相关库
from PySide6.QtCore import QObject

# 导入监视器 UI
from amr_monitor.ui.monitors.imu_monitor_ui import Ui_IMUMonitor


class IMUMonitor(QObject, Ui_IMUMonitor):
    """Cmd_Vel Monitor"""

    def __init__(self, parent=None):
        """初始化"""
        super().__init__(parent)
        self.setupUi(parent)

        # 初始化信号和槽
        self.connect_signals()

    def connect_signals(self):
        """连接信号"""

        pass
