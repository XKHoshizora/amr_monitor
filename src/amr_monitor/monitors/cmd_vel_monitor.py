# -*- coding: utf-8 -*-
"""Cmd_Vel Monitor
src/amr_monitor/monitors/cmd_vel_monitor.py
"""

# 导入 Qt 相关库
from PySide6.QtCore import QObject

# 导入监视器 UI
from amr_monitor.ui.monitors.cmd_vel_monitor_ui import Ui_CmdVelMonitor


class CmdVelMonitor(QObject, Ui_CmdVelMonitor):
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
