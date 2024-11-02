# src/amr_monitor/core/monitor_manager.py
"""监控器管理器"""
import threading
from typing import Dict, Any, Optional, Type
import time
from dataclasses import dataclass
from PySide6.QtCore import QObject, Signal

from amr_monitor.core.logger import Logger
from amr_monitor.core.event_bus import EventBus
from amr_monitor.core.error_handler import handle_error, ErrorType, ErrorSeverity
from amr_monitor.monitors.base_monitor import BaseMonitor

logger = Logger.get_logger(__name__)


@dataclass
class MonitorStatus:
    """监控器状态"""
    is_active: bool = False
    error_count: int = 0
    last_update: float = 0
    health_status: str = "unknown"
    reconnect_attempts: int = 0


class MonitorManager(QObject):
    """监控器管理器"""

    # 信号定义
    monitor_status_changed = Signal(str, str)  # monitor_name, status
    monitor_error = Signal(str, str)  # monitor_name, error_message
    sync_completed = Signal()

    def __init__(self, event_bus: EventBus):
        super().__init__()
        self.event_bus = event_bus
        self.monitors: Dict[str, BaseMonitor] = {}
        self.monitor_status: Dict[str, MonitorStatus] = {}
        self._lock = threading.Lock()
        self.max_reconnect_attempts = 3
        self.sync_interval = 1.0  # seconds

        # 订阅事件
        self.event_bus.subscribe('monitor_data', self._handle_monitor_data)
        self.event_bus.subscribe('error', self._handle_monitor_error)

    def register_monitor(self, name: str, monitor: BaseMonitor) -> bool:
        """注册监控器"""
        with self._lock:
            if name in self.monitors:
                return False

            # 添加类型检查
            if not isinstance(monitor, BaseMonitor):
                raise TypeError("monitor must be an instance of BaseMonitor")

            self.monitors[name] = monitor
            self.monitor_status[name] = MonitorStatus()

            # 连接监控器信号
            monitor.error_occurred.connect(
                lambda error: self._handle_monitor_error(name, error))

            logger.info(f"Registered monitor: {name}")
            return True

    def unregister_monitor(self, name: str):
        """注销监控器"""
        with self._lock:
            if name in self.monitors:
                monitor = self.monitors[name]
                monitor.cleanup()
                del self.monitors[name]
                del self.monitor_status[name]
                logger.info(f"Unregistered monitor: {name}")

    def activate_monitor(self, name: str) -> bool:
        """激活监控器"""
        with self._lock:
            if name not in self.monitors:
                return False

            monitor = self.monitors[name]
            status = self.monitor_status[name]

            # 检查当前状态
            if status.is_active:
                return True

            try:
                monitor.start()
                status.is_active = True
                status.last_update = time.time()
                status.health_status = "active"
                self.monitor_status_changed.emit(name, "active")
                return True
            except Exception as e:
                error_msg = f"Failed to activate monitor {name}: {str(e)}"
                handle_error(ErrorType.SYSTEM, ErrorSeverity.ERROR,
                             error_msg, "MonitorManager")
                return False

    def deactivate_monitor(self, name: str):
        """停用监控器"""
        with self._lock:
            if name in self.monitors:
                monitor = self.monitors[name]
                status = self.monitor_status[name]

                monitor.stop()
                status.is_active = False
                status.health_status = "inactive"
                self.monitor_status_changed.emit(name, "inactive")
                logger.info(f"Deactivated monitor: {name}")

    def sync_monitors(self):
        """同步监控器数据"""
        with self._lock:
            for name, monitor in self.monitors.items():
                status = self.monitor_status[name]
                if status.is_active:
                    try:
                        monitor.sync_data()
                    except Exception as e:
                        error_msg = f"Failed to sync monitor {name}: {str(e)}"
                        handle_error(ErrorType.SYSTEM, ErrorSeverity.WARNING,
                                     error_msg, "MonitorManager")
        self.sync_completed.emit()

    def _handle_monitor_data(self, event):
        """处理监控器数据"""
        monitor_name = event.source
        if monitor_name in self.monitor_status:
            status = self.monitor_status[monitor_name]
            status.last_update = time.time()
            status.health_status = "healthy"

    def _handle_monitor_error(self, monitor_name: str, error: str):
        """处理监控器错误"""
        with self._lock:
            if monitor_name in self.monitor_status:
                status = self.monitor_status[monitor_name]
                status.error_count += 1
                status.health_status = "error"

                # 发送错误信号
                self.monitor_error.emit(monitor_name, error)

                # 尝试恢复
                if status.reconnect_attempts < self.max_reconnect_attempts:
                    self._attempt_recovery(monitor_name)
                else:
                    self.deactivate_monitor(monitor_name)
                    error_msg = f"Monitor {monitor_name} failed after {status.reconnect_attempts} recovery attempts"
                    handle_error(ErrorType.SYSTEM, ErrorSeverity.ERROR,
                                 error_msg, "MonitorManager")

    def _attempt_recovery(self, monitor_name: str):
        """尝试恢复监控器"""
        if monitor_name not in self.monitor_status:
            return

        status = self.monitor_status[monitor_name]
        if status.reconnect_attempts >= self.max_reconnect_attempts:
            # 达到最大重试次数，停止监控器
            self.deactivate_monitor(monitor_name)
            error_msg = f"Monitor {monitor_name} failed after {self.max_reconnect_attempts} attempts"
            handle_error(ErrorType.SYSTEM, ErrorSeverity.ERROR,
                         error_msg, "MonitorManager")
            return

        status.reconnect_attempts += 1
        logger.info(f"Attempting to recover monitor {monitor_name} "
                    f"(attempt {status.reconnect_attempts}/{self.max_reconnect_attempts})")

        # 停止监控器
        self.deactivate_monitor(monitor_name)

        # 等待短暂时间
        time.sleep(0.5)  # 减少等待时间

        # 重新激活
        if self.activate_monitor(monitor_name):
            status.reconnect_attempts = 0
            status.error_count = 0
            logger.info(f"Successfully recovered monitor {monitor_name}")

    def get_monitor_status(self, name: str) -> Optional[MonitorStatus]:
        """获取监控器状态"""
        return self.monitor_status.get(name)

    def get_active_monitors(self) -> list:
        """获取活动的监控器列表"""
        return [name for name, status in self.monitor_status.items()
                if status.is_active]

    def cleanup(self):
        """清理所有监控器"""
        with self._lock:
            for name in list(self.monitors.keys()):
                self.deactivate_monitor(name)
                self.unregister_monitor(name)


# 创建全局实例
monitor_manager = MonitorManager(EventBus())
