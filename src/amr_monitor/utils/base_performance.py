"""基础性能监控模块"""
from typing import Dict, Any
import threading
import time
import psutil
import numpy as np
import rospy

class BasePerformanceMonitor:
    """基础性能监控类"""

    def __init__(self):
        self.metrics = {}
        self.start_time = time.time()
        self._lock = threading.Lock()
        self.process = psutil.Process()

        # 初始化指标配置
        self.metrics_config = {
            'cpu_usage': {
                'threshold': 80.0,
                'window_size': 100
            },
            'memory_usage': {
                'threshold': 80.0,
                'window_size': 100
            }
        }

        # 初始化指标缓冲区
        self._init_metrics_buffer()

    def _init_metrics_buffer(self):
        """初始化指标缓冲区"""
        for metric, config in self.metrics_config.items():
            self.metrics[metric] = {
                'values': np.zeros(config['window_size']),
                'timestamps': np.zeros(config['window_size']),
                'current_index': 0,
                'threshold': config['threshold']
            }

    def update_metric(self, name: str, value: float):
        """更新指标值"""
        if name not in self.metrics:
            return

        with self._lock:
            metric = self.metrics[name]
            idx = metric['current_index']

            metric['values'][idx] = value
            metric['timestamps'][idx] = time.time()
            metric['current_index'] = (idx + 1) % len(metric['values'])

            if value > metric['threshold']:
                self._handle_threshold_exceeded(name, value)

    def get_metric_stats(self, name: str) -> Dict[str, float]:
        """获取指标统计信息"""
        if name not in self.metrics:
            return {}

        with self._lock:
            values = self.metrics[name]['values']
            return {
                'current': values[-1],
                'mean': np.mean(values),
                'max': np.max(values),
                'min': np.min(values),
                'std': np.std(values)
            }

    def get_system_metrics(self) -> Dict[str, Any]:
        """获取系统指标"""
        return {
            'cpu_percent': self.process.cpu_percent(),
            'memory_percent': self.process.memory_percent(),
            'num_threads': self.process.num_threads(),
            'uptime': time.time() - self.start_time
        }

    def reset_metrics(self):
        """重置所有指标"""
        with self._lock:
            self._init_metrics_buffer()

    def _handle_threshold_exceeded(self, metric_name: str, value: float):
        """处理超过阈值的情况"""
        rospy.logwarn(f"Metric {metric_name} exceeded threshold: {value}")