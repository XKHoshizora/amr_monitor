#!/usr/bin/env python3
"""AMR Monitor节点"""
import sys
from pathlib import Path
import signal
import rospy
from PySide6.QtWidgets import QApplication
from PySide6.QtCore import Qt

from amr_monitor.core.config import ConfigManager
from amr_monitor.core.event_bus import EventBus, Event
from amr_monitor.core.logger import Logger, LoggerManager
from amr_monitor.utils.storage import StorageManager
from amr_monitor.main_window import MainWindow


class AMRMonitorNode:
    """AMR Monitor节点类"""

    def __init__(self):
        """初始化节点"""
        # 初始化ROS节点
        rospy.init_node('amr_monitor', anonymous=True)

        # 初始化日志系统
        self.logger_manager = LoggerManager()
        self.logger = Logger.get_logger('amr_monitor')
        self.logger_manager.start_logging()

        try:
            # 初始化核心组件
            self.config_manager = ConfigManager()
            self.event_bus = EventBus()

            # 获取配置
            storage_config = self.config_manager.get_config('storage', {})
            enable_storage = rospy.get_param('~enable_data_storage', True)

            # 初始化数据存储(如果启用)
            self.storage_manager = None
            if enable_storage:
                self.storage_manager = StorageManager(storage_config)
                self.logger.info("Data storage enabled")

            # 创建Qt应用
            self.app = QApplication(sys.argv)

            # 设置样式
            self._setup_application_style()

            # 创建并显示主窗口
            self.window = MainWindow(
                config_manager=self.config_manager,
                event_bus=self.event_bus,
                storage_manager=self.storage_manager
            )
            self.window.show()

            # 注册信号处理
            signal.signal(signal.SIGINT, self.signal_handler)

            # 注册ROS关闭回调
            rospy.on_shutdown(self.shutdown_hook)

            # 订阅事件
            self._setup_event_handlers()

            self.logger.info("AMR Monitor node initialized")

        except Exception as e:
            self.logger.error(f"Failed to initialize AMR Monitor node: {e}")
            raise

    def _setup_application_style(self):
        """设置应用样式"""
        # 获取主题设置
        theme = rospy.get_param('~theme', 'dark')

        # 设置应用程序属性
        self.app.setAttribute(Qt.AA_EnableHighDpiScaling)
        self.app.setStyle('Fusion')

        # 加载主题
        theme_file = self.config_manager.get_theme_file(theme)
        if theme_file and theme_file.exists():
            with open(theme_file) as f:
                self.app.setStyleSheet(f.read())

    def _setup_event_handlers(self):
        """设置事件处理器"""
        # 监控数据事件
        self.event_bus.subscribe('monitor_data', self._handle_monitor_data)

        # 错误事件
        self.event_bus.subscribe('error', self._handle_error)

        # 性能事件
        self.event_bus.subscribe('performance', self._handle_performance)

    def _handle_monitor_data(self, event: Event):
        """处理监控数据事件"""
        if self.storage_manager:
            try:
                self.storage_manager.store_data(
                    event.source,
                    'monitor_data',
                    event.data
                )
            except Exception as e:
                self.logger.error(f"Failed to store monitor data: {e}")

    def _handle_error(self, event: Event):
        """处理错误事件"""
        self.logger.error(f"Error from {event.source}: {event.data}")

    def _handle_performance(self, event: Event):
        """处理性能事件"""
        if event.data.get('cpu_percent', 0) > 80 or event.data.get('memory_percent', 0) > 80:
            self.logger.warning(
                f"High resource usage from {event.source}: {event.data}")

    def signal_handler(self, signum, frame):
        """信号处理器"""
        self.logger.info(f"Received signal {signum}")
        self.cleanup()
        sys.exit(0)

    def shutdown_hook(self):
        """ROS关闭钩子"""
        self.logger.info("Shutting down AMR Monitor node")
        self.cleanup()

    def cleanup(self):
        """清理资源"""
        try:
            # 关闭存储管理器
            if self.storage_manager:
                self.storage_manager.close()

            # 需要等待写入线程完成
            if hasattr(self.storage_manager, '_write_thread'):
                self.storage_manager._write_thread.join(timeout=5.0)

            # 清理事件总线
            self.event_bus.clear()

            # 停止日志系统
            self.logger_manager.stop_logging()

            # 关闭Qt应用
            self.app.quit()

        except Exception as e:
            self.logger.error(f"Error during cleanup: {e}")

    def run(self):
        """运行节点"""
        try:
            # 运行Qt应用
            status = self.app.exec_()

            # 清理资源
            self.cleanup()

            return status

        except Exception as e:
            self.logger.error(f"Error running AMR Monitor node: {e}")
            self.cleanup()
            return 1


def main():
    """主函数"""
    try:
        node = AMRMonitorNode()
        sys.exit(node.run())
    except Exception as e:
        rospy.logerr(f"AMR Monitor node failed: {e}")
        sys.exit(1)


if __name__ == '__main__':
    main()
