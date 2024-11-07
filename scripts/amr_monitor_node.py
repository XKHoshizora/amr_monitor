# scripts/amr_monitor_node.py
# -*- coding: utf-8 -*-
"""AMR Monitor节点"""

# 导入标准库
import sys
from pathlib import Path

# 导入 Qt 相关库
from PySide6.QtWidgets import QApplication
from PySide6.QtCore import Qt

# 获取项目根目录路径
project_root = Path(__file__).resolve().parents[1]

# 将 src 目录添加到 sys.path 中
src_path = project_root / "src"
sys.path.append(str(src_path))

# 导入核心包
from amr_monitor.main_window import MainWindow


class RunGUI(MainWindow):
    def __init__(self):
        super().__init__()


if __name__ == "__main__":
    app = QApplication(sys.argv)
    
    gui = RunGUI()
    gui.show()

    app.exec()
