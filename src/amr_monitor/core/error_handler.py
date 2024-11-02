"""错误处理系统实现"""
from enum import Enum, auto
from typing import Optional, Dict, Any
import traceback
from datetime import datetime
from PySide6.QtCore import QObject, Signal
import rospy

from .logger import Logger

logger = Logger.get_logger(__name__)

class ErrorSeverity(Enum):
    """错误严重程度"""
    INFO = auto()
    WARNING = auto()
    ERROR = auto()
    CRITICAL = auto()

class ErrorType(Enum):
    """错误类型"""
    TOPIC = auto()    # 话题相关错误
    DATA = auto()     # 数据处理错误
    SYSTEM = auto()   # 系统错误
    UI = auto()       # UI相关错误

class ErrorEvent:
    """错误事件"""
    def __init__(self,
                 error_type: ErrorType,
                 severity: ErrorSeverity,
                 message: str,
                 source: str,
                 details: Optional[Dict[str, Any]] = None):
        self.error_type = error_type
        self.severity = severity
        self.message = message
        self.source = source
        self.details = details or {}
        self.timestamp = datetime.now()
        self.traceback = traceback.extract_stack()

    def __str__(self):
        return (f"[{self.timestamp:%Y-%m-%d %H:%M:%S}] "
                f"{self.severity.name}: {self.message} "
                f"(Source: {self.source})")

class ErrorHandler(QObject):
    """错误处理器"""
    error_occurred = Signal(ErrorEvent)  # 错误发生信号

    _instance = None

    def __new__(cls):
        if cls._instance is None:
            cls._instance = super().__new__(cls)
        return cls._instance

    def __init__(self):
        if not hasattr(self, '_initialized'):
            super().__init__()
            self._initialized = True
            self.error_history = []
            self.max_history = 100

    def handle_error(self, error_event: ErrorEvent):
        """处理错误"""
        # 记录错误
        self.error_history.append(error_event)
        if len(self.error_history) > self.max_history:
            self.error_history.pop(0)

        # 记录日志
        if error_event.severity == ErrorSeverity.CRITICAL:
            logger.critical(str(error_event))
        elif error_event.severity == ErrorSeverity.ERROR:
            logger.error(str(error_event))
        elif error_event.severity == ErrorSeverity.WARNING:
            logger.warning(str(error_event))
        else:
            logger.info(str(error_event))

        # 发送错误信号
        self.error_occurred.emit(error_event)

        # ROS日志
        self._log_to_ros(error_event)

    def _log_to_ros(self, error_event: ErrorEvent):
        """记录到ROS日志"""
        msg = f"{error_event.message} (Source: {error_event.source})"
        if error_event.severity == ErrorSeverity.CRITICAL:
            rospy.logfatal(msg)
        elif error_event.severity == ErrorSeverity.ERROR:
            rospy.logerr(msg)
        elif error_event.severity == ErrorSeverity.WARNING:
            rospy.logwarn(msg)
        else:
            rospy.loginfo(msg)

    def get_error_history(self) -> list:
        """获取错误历史"""
        return self.error_history

    def clear_history(self):
        """清空错误历史"""
        self.error_history.clear()

# 创建全局错误处理器实例
error_handler = ErrorHandler()

def handle_error(error_type: ErrorType,
                severity: ErrorSeverity,
                message: str,
                source: str,
                details: Optional[Dict[str, Any]] = None):
    """便捷的错误处理函数"""
    error_event = ErrorEvent(error_type, severity, message, source, details)
    error_handler.handle_error(error_event)