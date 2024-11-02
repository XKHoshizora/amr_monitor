"""状态显示控件"""
from PySide6.QtWidgets import QLabel
from PySide6.QtCore import Qt
from .base_widget import BaseWidget


class StatusWidget(BaseWidget):
    """状态显示控件"""

    def __init__(self, parent=None):
        self.status_label = None
        super().__init__(parent)

    def setup_ui(self):
        """设置UI"""
        self.status_label = QLabel()
        self.status_label.setAlignment(Qt.AlignCenter)
        self.layout.addWidget(self.status_label)

    def set_status(self, status: str):
        """设置状态文本"""
        self.status_label.setText(status)
        self.update_state(status)

    def set_style(self, color: str):
        """设置状态样式"""
        self.status_label.setStyleSheet(f"color: {color};")
