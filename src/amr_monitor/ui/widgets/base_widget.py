"""基础UI控件"""
from PySide6.QtWidgets import QWidget, QVBoxLayout
from PySide6.QtCore import Signal

class BaseWidget(QWidget):
    """基础控件"""
    # 通用信号
    state_changed = Signal(str)  # 状态变更信号
    error_occurred = Signal(str)  # 错误信号

    def __init__(self, parent=None):
        super().__init__(parent)
        self.layout = QVBoxLayout(self)
        self.setup_ui()

    def setup_ui(self):
        """设置UI"""
        pass

    def update_state(self, state: str):
        """更新状态"""
        self.state_changed.emit(state)

    def handle_error(self, error: str):
        """处理错误"""
        self.error_occurred.emit(error)