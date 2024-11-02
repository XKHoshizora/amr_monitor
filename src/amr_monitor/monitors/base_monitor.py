# src/amr_monitor/monitors/base_monitor.py
"""基础监控器模块"""
import threading
import time
from abc import ABC, abstractmethod
from collections import deque
from typing import Dict, Any, Optional
import numpy as np
from queue import Queue
import rospy
from PySide6.QtWidgets import (QWidget, QVBoxLayout, QHBoxLayout, QLabel,
                               QComboBox, QPushButton, QFrame)
from PySide6.QtCore import QTimer, Signal

from amr_monitor.core.event_bus import EventBus, Event
from amr_monitor.core.logger import Logger
from amr_monitor.core.error_handler import handle_error, ErrorType, ErrorSeverity
from amr_monitor.core.topic_manager import topic_manager, TopicSubscription
from amr_monitor.utils.base_performance import BasePerformanceMonitor
from amr_monitor.ui.widgets.plot_widget import PlotWidget
from amr_monitor.ui.widgets.data_table import DataTableWidget
from amr_monitor.ui.widgets.status_widget import StatusWidget

logger = Logger.get_logger(__name__)


class DataProcessor(ABC):
    """数据处理基类"""

    # 添加信号
    error_occurred = Signal(str, str)  # monitor_name, error_message

    def __init__(self, config: Dict[str, Any], event_bus: EventBus, parent=None):
        buffer_size = config.get('buffer_size', 1000)

        self.data_buffer = Queue(maxsize=buffer_size)
        self.timestamps = deque(maxlen=buffer_size)
        self.statistics = {}
        self.is_active = False
        self.cleanup_lock = threading.Lock()
        self.topic_subscription = None

    @abstractmethod
    def process(self, data: Any) -> Dict[str, Any]:
        """处理原始数据"""
        pass

    def update(self, data: Any):
        """更新数据"""
        processed_data = self.process(data)

        if self.data_buffer.full():
            self.data_buffer.get()
        self.data_buffer.put(processed_data)

        self.timestamps.append(time.time())
        self._update_statistics(processed_data)

        return processed_data

    def _update_statistics(self, data: Dict[str, Any]):
        """更新统计信息"""
        for key, value in data.items():
            if isinstance(value, (int, float)):  # 只处理数值类型
                if key not in self.statistics:
                    self.statistics[key] = {
                        'count': 0,
                        'mean': 0,
                        'min': float('inf'),
                        'max': float('-inf')
                    }

                stats = self.statistics[key]
                stats['count'] += 1
                stats['mean'] = (
                    stats['mean'] * (stats['count'] - 1) + value) / stats['count']
                stats['min'] = min(stats['min'], value)
                stats['max'] = max(stats['max'], value)

    def get_data(self) -> Dict[str, Any]:
        """获取最新数据"""
        return self.data_buffer.queue[-1] if not self.data_buffer.empty() else {}

    def get_buffer_data(self) -> list:
        """获取缓冲区数据"""
        return list(self.data_buffer.queue)

    def get_statistics(self) -> Dict[str, Dict]:
        """获取统计信息"""
        return self.statistics

    def clear(self):
        """清空数据"""
        while not self.data_buffer.empty():
            self.data_buffer.get()
        self.timestamps.clear()
        self.statistics.clear()


class BaseMonitor(QWidget):
    """基础监控器类"""

    def __init__(self, config: Dict[str, Any], event_bus: EventBus, parent=None):
        super().__init__(parent)

        self.config = config
        self.event_bus = event_bus
        self.performance_monitor = BasePerformanceMonitor()
        self.data_processor = None

        # 更新频率设置
        self.min_update_interval = 0.01
        self.max_update_interval = 1.0
        self.current_update_interval = self.config.get('update_interval', 0.1)

        # UI组件
        self.topic_combobox = None
        self.refresh_button = None
        self.status_label = None
        self.plot_frame = None
        self.info_frame = None
        self.plots = {}
        self.data_tables = {}

        self.setup_ui()
        self._setup_update_timer()

    def setup_ui(self):
        """设置UI"""
        main_layout = QVBoxLayout(self)

        # 控制面板
        control_frame = QFrame(self)
        control_layout = QHBoxLayout(control_frame)

        self.topic_combobox = QComboBox()
        self.topic_combobox.setMinimumWidth(200)

        self.refresh_button = QPushButton("Refresh")
        self.refresh_button.clicked.connect(self._on_refresh_clicked)

        self.status_label = QLabel("Status: Not Connected")

        control_layout.addWidget(QLabel("Topic:"))
        control_layout.addWidget(self.topic_combobox)
        control_layout.addWidget(self.refresh_button)
        control_layout.addWidget(self.status_label)
        control_layout.addStretch()

        main_layout.addWidget(control_frame)

        # 绘图区域
        self.plot_frame = QFrame()
        self.plot_frame.setLayout(QVBoxLayout())
        main_layout.addWidget(self.plot_frame)

        # 信息显示区域
        self.info_frame = QFrame()
        self.info_frame.setLayout(QHBoxLayout())
        main_layout.addWidget(self.info_frame)

    def _setup_update_timer(self):
        """设置更新定时器"""
        self.update_timer = QTimer(self)
        self.update_timer.timeout.connect(self.update_displays)
        self.update_timer.start(int(self.current_update_interval * 1000))

    def _on_refresh_clicked(self):
        """刷新按钮点击处理"""
        pass  # 子类实现具体逻辑

    def add_plot(self, name: str, title: str = "", x_label: str = "", y_label: str = ""):
        """添加绘图"""
        plot = PlotWidget(title)
        plot.set_labels(x_label, y_label)
        self.plots[name] = plot
        self.plot_frame.layout().addWidget(plot)

    def update_displays(self):
        """更新显示"""
        if not self.data_processor:
            return

        try:
            data = self.data_processor.get_data()
            if data:
                self._update_plots(data)
                self._update_info(data)
                self._update_status(data)
        except Exception as e:
            self.handle_error(f"Display update error: {str(e)}")

    @abstractmethod
    def _update_plots(self, data: Dict[str, Any]):
        """更新图表"""
        pass

    @abstractmethod
    def _update_info(self, data: Dict[str, Any]):
        """更新信息显示"""
        pass

    def _update_status(self, data: Dict[str, Any]):
        """更新状态"""
        status_text = self.get_status_text(data)
        self.status_label.setText(status_text)

    @abstractmethod
    def get_status_text(self, data: Dict[str, Any]) -> str:
        """获取状态文本"""
        pass

    def handle_error(self, error: str):
        """处理错误"""
        logger.error(f"{self.__class__.__name__}: {error}")
        self.error_occurred.emit(self.__class__.__name__, error)

    def start(self):
        """启动监控"""
        if not self.is_active:
            self.is_active = True
            self.update_timer.start()
            # 重新订阅话题
            self.setup_subscriber()
            logger.info(f"{self.__class__.__name__} started")

    def stop(self):
        """停止监控"""
        if self.is_active:
            self.is_active = False
            self.update_timer.stop()
            # 取消话题订阅
            if hasattr(self, 'topic_sub') and self.topic_sub:
                self.topic_sub.unregister()
                self.topic_sub = None
            logger.info(f"{self.__class__.__name__} stopped")

    def clear_data(self):
        """清空数据"""
        if self.data_processor:
            self.data_processor.clear()
        for plot in self.plots.values():
            plot.clear_all()

    def export_data(self, filename: Optional[str] = None) -> bool:
        """导出数据"""
        if not self.data_processor:
            return False

        try:
            if filename is None:
                filename = f"{self.__class__.__name__}_{time.strftime('%Y%m%d_%H%M%S')}.csv"

            import pandas as pd
            data = self.data_processor.get_buffer_data()
            if not data:
                return False

            df = pd.DataFrame(data)
            df['timestamp'] = list(self.data_processor.timestamps)
            df.to_csv(filename, index=False)
            return True
        except Exception as e:
            self.handle_error(f"Data export error: {str(e)}")
            return False

    def setup_subscriber(self):
        """设置话题订阅"""
        if not hasattr(self, 'get_message_type'):
            raise NotImplementedError(
                "Monitor must implement get_message_type()")

        topic = self.topic_combobox.currentText() or self.config.get('topic')
        if not topic:
            self.handle_error("No topic specified")
            return

        try:
            self.topic_subscription = TopicSubscription(
                topic_name=topic,
                msg_type=self.get_message_type(),
                callback=self._topic_callback,
                monitor_name=self.__class__.__name__,
                queue_size=self.config.get('queue_size', 10)
            )

            if topic_manager.subscribe(self.topic_subscription):
                self.status_label.setText(f"Connected to {topic}")
            else:
                self.status_label.setText(f"Failed to connect to {topic}")

        except Exception as e:
            self.handle_error(f"Failed to setup subscriber: {str(e)}")

    def _topic_callback(self, msg):
        """话题回调的包装器"""
        try:
            data = self.process_message(msg)
            if data:
                self.event_bus.publish(Event(
                    'monitor_data',
                    data,
                    self.__class__.__name__
                ))
        except Exception as e:
            self.handle_error(f"Message processing error: {str(e)}")

    @abstractmethod
    def process_message(self, msg) -> Optional[Dict[str, Any]]:
        """处理接收到的消息"""
        pass

    @abstractmethod
    def get_message_type(self):
        """获取消息类型"""
        pass

    def cleanup(self):
        """清理资源"""
        with self.cleanup_lock:
            if self.topic_subscription:
                topic_manager.unsubscribe(self.topic_subscription.topic_name)
                self.topic_subscription = None
            self.stop()
            self.clear_data()

    def _on_refresh_clicked(self):
        """刷新按钮点击处理"""
        try:
            msg_type = self.get_message_type().__name__
            available_topics = topic_manager.get_available_topics(msg_type)

            current = self.topic_combobox.currentText()
            self.topic_combobox.clear()
            self.topic_combobox.addItems(available_topics)

            index = self.topic_combobox.findText(current)
            if index >= 0:
                self.topic_combobox.setCurrentIndex(index)

            self.setup_subscriber()

        except Exception as e:
            self.handle_error(f"Failed to refresh topics: {str(e)}")
