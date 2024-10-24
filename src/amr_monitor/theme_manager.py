from PyQt5.QtWidgets import *
from PyQt5.QtCore import *
from PyQt5.QtGui import *


class ThemeManager:
    """主题管理器"""
    @staticmethod
    def apply_dark_theme(app):
        """应用暗色主题"""
        app.setStyle("Fusion")

        # 创建暗色调色板
        dark_palette = QPalette()

        # 设置基础颜色
        dark_palette.setColor(QPalette.Window, QColor(53, 53, 53))
        dark_palette.setColor(QPalette.WindowText, QColor(255, 255, 255))
        dark_palette.setColor(QPalette.Base, QColor(35, 35, 35))
        dark_palette.setColor(QPalette.AlternateBase, QColor(53, 53, 53))
        dark_palette.setColor(QPalette.ToolTipBase, QColor(25, 25, 25))
        dark_palette.setColor(QPalette.ToolTipText, QColor(255, 255, 255))
        dark_palette.setColor(QPalette.Text, QColor(255, 255, 255))
        dark_palette.setColor(QPalette.Button, QColor(53, 53, 53))
        dark_palette.setColor(QPalette.ButtonText, QColor(255, 255, 255))
        dark_palette.setColor(QPalette.BrightText, QColor(255, 0, 0))
        dark_palette.setColor(QPalette.Link, QColor(42, 130, 218))
        dark_palette.setColor(QPalette.Highlight, QColor(42, 130, 218))
        dark_palette.setColor(QPalette.HighlightedText, QColor(35, 35, 35))

        # 应用暗色主题
        app.setPalette(dark_palette)

        # 设置样式表
        app.setStyleSheet("""
            QToolTip {
                color: #ffffff;
                background-color: #2a82da;
                border: 1px solid #000000;
                padding: 2px;
            }
            QGroupBox {
                border: 1px solid #666666;
                border-radius: 4px;
                margin-top: 0.5em;
                padding-top: 0.5em;
            }
            QGroupBox::title {
                subcontrol-origin: margin;
                left: 10px;
                padding: 0 3px 0 3px;
            }
        """)

    @staticmethod
    def apply_light_theme(app):
        """应用亮色主题"""
        app.setStyle("Fusion")
        app.setPalette(app.style().standardPalette())

        # 设置样式表
        app.setStyleSheet("""
            QGroupBox {
                border: 1px solid #cccccc;
                border-radius: 4px;
                margin-top: 0.5em;
                padding-top: 0.5em;
            }
            QGroupBox::title {
                subcontrol-origin: margin;
                left: 10px;
                padding: 0 3px 0 3px;
            }
        """)
