# src/amr_monitor/core/logger.py
import os
import sys
import logging
import logging.handlers
import threading
from datetime import datetime
from pathlib import Path
from typing import Optional, Dict
import rospy
from functools import wraps

class LoggerSingleton(type):
    """Logger单例元类"""
    _instances: Dict[str, logging.Logger] = {}
    _lock = threading.Lock()

    def __call__(cls, name: str = None) -> logging.Logger:
        with cls._lock:
            if name not in cls._instances:
                instance = super().__call__(name)
                cls._instances[name] = instance
            return cls._instances[name]

class Logger(metaclass=LoggerSingleton):
    """增强的日志管理器"""

    LOG_LEVELS = {
        'debug': logging.DEBUG,
        'info': logging.INFO,
        'warning': logging.WARNING,
        'error': logging.ERROR,
        'critical': logging.CRITICAL
    }

    def __init__(self, name: str = None):
        """初始化日志管理器

        Args:
            name (str, optional): 日志器名称. Defaults to None.
        """
        self.name = name or 'amr_monitor'
        self.logger = self._setup_logger()

    def _setup_logger(self) -> logging.Logger:
        """设置日志器

        Returns:
            logging.Logger: 配置好的日志器
        """
        # 创建日志器
        logger = logging.getLogger(self.name)
        logger.setLevel(logging.DEBUG)

        # 避免重复处理器
        if logger.handlers:
            return logger

        try:
            # 创建日志目录
            log_dir = Path.home() / '.ros' / 'amr_monitor' / 'logs'
            log_dir.mkdir(parents=True, exist_ok=True)

            # 文件处理器
            log_file = log_dir / f"{self.name}_{datetime.now():%Y%m%d}.log"
            file_handler = logging.handlers.RotatingFileHandler(
                filename=log_file,
                maxBytes=10*1024*1024,  # 10MB
                backupCount=5,
                encoding='utf-8'
            )
            file_handler.setLevel(logging.DEBUG)

            # 控制台处理器
            console_handler = logging.StreamHandler(sys.stdout)
            console_handler.setLevel(logging.INFO)

            # 创建格式器
            detailed_formatter = logging.Formatter(
                '%(asctime)s - %(name)s - %(levelname)s - %(pathname)s:%(lineno)d - %(message)s'
            )
            simple_formatter = logging.Formatter(
                '%(asctime)s - %(levelname)s - %(message)s'
            )

            file_handler.setFormatter(detailed_formatter)
            console_handler.setFormatter(simple_formatter)

            # 添加处理器
            logger.addHandler(file_handler)
            logger.addHandler(console_handler)

            # ROS日志处理器
            if rospy.get_name() != '/unnamed':
                logger.addHandler(ROSLogHandler())

            return logger

        except Exception as e:
            # 确保至少有基本的日志功能
            fallback_handler = logging.StreamHandler(sys.stderr)
            fallback_handler.setFormatter(simple_formatter)
            logger.addHandler(fallback_handler)
            logger.error(f"Failed to setup logger: {e}")
            return logger

    @classmethod
    def get_logger(cls, name: Optional[str] = None) -> logging.Logger:
        """获取日志器实例

        Args:
            name (str, optional): 日志器名称. Defaults to None.

        Returns:
            logging.Logger: 日志器实例
        """
        return cls(name).logger

    @classmethod
    def set_level(cls, level: str):
        """设置日志级别

        Args:
            level (str): 日志级别名称
        """
        if level.lower() in cls.LOG_LEVELS:
            for logger in cls._instances.values():
                logger.setLevel(cls.LOG_LEVELS[level.lower()])

class ROSLogHandler(logging.Handler):
    """ROS日志处理器"""

    def __init__(self):
        super().__init__()
        self.ros_level_map = {
            logging.DEBUG: rospy.logdebug,
            logging.INFO: rospy.loginfo,
            logging.WARNING: rospy.logwarn,
            logging.ERROR: rospy.logerr,
            logging.CRITICAL: rospy.logfatal
        }

    def emit(self, record):
        try:
            if not rospy.is_shutdown():
                msg = self.format(record)
                log_func = self.ros_level_map.get(record.levelno, rospy.loginfo)
                log_func(msg)
        except Exception:
            self.handleError(record)

def log_exceptions(func):
    """异常日志装饰器"""
    @wraps(func)
    def wrapper(*args, **kwargs):
        logger = Logger.get_logger()
        try:
            return func(*args, **kwargs)
        except Exception as e:
            logger.exception(f"Exception in {func.__name__}: {str(e)}")
            raise
    return wrapper

class LoggerManager:
    """日志管理器"""

    def __init__(self):
        self.logger = Logger.get_logger()
        self._setup_exception_hook()

    def _setup_exception_hook(self):
        """设置全局异常钩子"""
        def exception_hook(exctype, value, traceback):
            if issubclass(exctype, KeyboardInterrupt):
                sys.__excepthook__(exctype, value, traceback)
                return
            self.logger.error("Uncaught exception:", exc_info=(exctype, value, traceback))

        sys.excepthook = exception_hook

    def start_logging(self):
        """启动日志记录"""
        self.logger.info("Starting AMR Monitor logging system")

        # 记录系统信息
        self.logger.info(f"Python version: {sys.version}")
        self.logger.info(f"ROS version: {rospy.get_param('/rosdistro', 'unknown')}")
        self.logger.info(f"Node name: {rospy.get_name()}")
        self.logger.info(f"Log directory: {Path.home() / '.ros' / 'amr_monitor' / 'logs'}")

    def stop_logging(self):
        """停止日志记录"""
        self.logger.info("Stopping AMR Monitor logging system")
        logging.shutdown()

# 用法示例
if __name__ == '__main__':
    # 初始化ROS节点
    rospy.init_node('logger_test', anonymous=True)

    # 获取日志器
    logger = Logger.get_logger('test')

    # 使用装饰器
    @log_exceptions
    def test_function():
        raise ValueError("Test exception")

    # 测试日志记录
    logger.debug("Debug message")
    logger.info("Info message")
    logger.warning("Warning message")
    logger.error("Error message")

    try:
        test_function()
    except Exception:
        pass

    # 使用日志管理器
    log_manager = LoggerManager()
    log_manager.start_logging()

    rospy.spin()

    log_manager.stop_logging()