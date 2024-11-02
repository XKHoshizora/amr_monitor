# src/amr_monitor/monitors/lidar_monitor.py
"""激光雷达监控器"""
import numpy as np
from typing import Dict, Any
import rospy
from sensor_msgs.msg import LaserScan

from amr_monitor.core.logger import Logger
from amr_monitor.core.event_bus import EventBus
from .base_monitor import BaseMonitor, DataProcessor

logger = Logger.get_logger(__name__)


class LidarDataProcessor(DataProcessor):
    """激光雷达数据处理器"""

    def __init__(self, buffer_size: int = 500):
        super().__init__(buffer_size)
        self.start_time = None
        self.last_scan_time = None
        self.scan_count = 0

    def process(self, msg: LaserScan) -> Dict[str, Any]:
        """处理激光雷达数据

        Args:
            msg (LaserScan): ROS激光雷达消息

        Returns:
            Dict[str, Any]: 处理后的数据字典
        """
        # 记录开始时间
        if self.start_time is None:
            self.start_time = msg.header.stamp.to_sec()

        # 计算扫描频率
        current_time = msg.header.stamp.to_sec()
        scan_frequency = 0.0
        if self.last_scan_time is not None:
            time_diff = current_time - self.last_scan_time
            if time_diff > 0:
                scan_frequency = 1.0 / time_diff
        self.last_scan_time = current_time

        # 转换为numpy数组以便处理
        ranges = np.array(msg.ranges)
        angles = np.linspace(
            msg.angle_min,
            msg.angle_max,
            len(ranges)
        )

        # 获取有效数据的掩码
        valid_mask = (ranges >= msg.range_min) & (ranges <= msg.range_max)
        valid_ranges = ranges[valid_mask]
        valid_angles = angles[valid_mask]

        # 计算直角坐标
        x = valid_ranges * np.cos(valid_angles)
        y = valid_ranges * np.sin(valid_angles)

        # 计算统计信息
        if len(valid_ranges) > 0:
            range_stats = {
                'min': float(np.min(valid_ranges)),
                'max': float(np.max(valid_ranges)),
                'mean': float(np.mean(valid_ranges)),
                'std': float(np.std(valid_ranges))
            }
        else:
            range_stats = {
                'min': 0.0,
                'max': 0.0,
                'mean': 0.0,
                'std': 0.0
            }

        # 处理强度数据(如果有)
        intensities = None
        if msg.intensities:
            intensities = np.array(msg.intensities)[valid_mask]

        self.scan_count += 1

        return {
            'x': x.tolist(),  # 转换为list以便JSON序列化
            'y': y.tolist(),
            'ranges': valid_ranges.tolist(),
            'angles': valid_angles.tolist(),
            'intensities': intensities.tolist() if intensities is not None else None,
            'stats': {
                'range': range_stats,
                'valid_points': len(valid_ranges),
                'total_points': len(ranges),
                'field_of_view': np.degrees(msg.angle_max - msg.angle_min),
                'angle_increment': np.degrees(msg.angle_increment),
                'scan_time': msg.scan_time,
                'scan_count': self.scan_count,
                'frequency': scan_frequency
            },
            'timestamp': current_time,
            'duration': current_time - self.start_time
        }


class LidarMonitor(BaseMonitor):
    """激光雷达监控器"""

    def __init__(self, config: Dict[str, Any], event_bus: EventBus, parent=None):
        super().__init__(config, event_bus, parent)

        # 创建数据处理器
        self.data_processor = LidarDataProcessor(
            buffer_size=config.get('buffer_size', 500)
        )

        # 设置图表
        self.setup_plots()

        # 设置ROS话题订阅
        self.topic_sub = None
        self.setup_subscriber()

    def setup_plots(self):
        """设置图表"""
        # 扫描数据极坐标图
        self.add_plot(
            'scan_polar',
            title='Laser Scan (Polar)',
            x_label='Angle (deg)',
            y_label='Range (m)'
        )

        # 扫描数据直角坐标图
        self.add_plot(
            'scan_cartesian',
            title='Laser Scan (Cartesian)',
            x_label='X (m)',
            y_label='Y (m)'
        )
        self.plots['scan_cartesian'].plot.setAspectLocked(True)

        # 强度数据图(如果有)
        self.add_plot(
            'intensity',
            title='Intensity',
            x_label='Angle (deg)',
            y_label='Intensity'
        )

    def setup_subscriber(self):
        """设置话题订阅"""
        if self.topic_sub:
            self.topic_sub.unregister()

        topic = self.topic_combobox.currentText() or self.config.get('topic', '/scan')
        try:
            self.topic_sub = rospy.Subscriber(
                topic,
                LaserScan,
                self.scan_callback,
                queue_size=5
            )
            self.status_label.setText(f"Connected to {topic}")
            logger.info(f"Subscribed to laser scan topic: {topic}")
        except Exception as e:
            self.handle_error(f"Failed to subscribe to {topic}: {e}")

    def scan_callback(self, msg: LaserScan):
        """激光雷达扫描回调处理"""
        try:
            # 处理数据
            data = self.data_processor.process(msg)

            # 发布事件
            self.event_bus.publish(Event(
                'monitor_data',
                data,
                self.__class__.__name__
            ))

        except Exception as e:
            self.handle_error(f"Laser scan processing error: {e}")

    def _update_plots(self, data: Dict[str, Any]):
        """更新图表显示"""
        # 更新极坐标图
        self.plots['scan_polar'].clear_all()
        self.plots['scan_polar'].update_curve(
            'range',
            np.degrees(data['angles']),  # 转换为角度
            data['ranges']
        )

        # 更新直角坐标图
        self.plots['scan_cartesian'].clear_all()
        self.plots['scan_cartesian'].update_curve(
            'points',
            data['x'],
            data['y']
        )

        # 更新强度图(如果有数据)
        if data['intensities'] is not None:
            self.plots['intensity'].clear_all()
            self.plots['intensity'].update_curve(
                'intensity',
                np.degrees(data['angles']),
                data['intensities']
            )

    def _update_info(self, data: Dict[str, Any]):
        """更新信息显示"""
        stats = data['stats']
        range_stats = stats['range']

        info_text = (
            f"Scan Info:\n"
            f"FOV: {stats['field_of_view']:.1f}°, "
            f"Resolution: {stats['angle_increment']:.2f}°\n"
            f"Range: min={range_stats['min']:.2f}m, "
            f"max={range_stats['max']:.2f}m, "
            f"avg={range_stats['mean']:.2f}m\n"
            f"Points: {stats['valid_points']}/{stats['total_points']}, "
            f"Frequency: {stats['frequency']:.1f}Hz"
        )

        for widget in self.info_frame.findChildren(QLabel):
            widget.setText(info_text)

    def get_status_text(self, data: Dict[str, Any]) -> str:
        """获取状态文本"""
        stats = data['stats']
        return (f"Points: {stats['valid_points']}/{stats['total_points']}, "
                f"Freq: {stats['frequency']:.1f}Hz")

    def _on_refresh_clicked(self):
        """刷新按钮点击处理"""
        try:
            # 获取当前可用的激光雷达话题
            scan_topics = [
                topic for topic, msg_type in rospy.get_published_topics()
                if msg_type == 'sensor_msgs/LaserScan'
            ]

            # 更新话题列表
            current = self.topic_combobox.currentText()
            self.topic_combobox.clear()
            self.topic_combobox.addItems(scan_topics)

            # 尝试恢复之前选择的话题
            index = self.topic_combobox.findText(current)
            if index >= 0:
                self.topic_combobox.setCurrentIndex(index)

            # 重新设置订阅
            self.setup_subscriber()

        except Exception as e:
            self.handle_error(f"Failed to refresh topics: {e}")

    def clear_data(self):
        """清空数据"""
        super().clear_data()
        if self.data_processor:
            self.data_processor.start_time = None
            self.data_processor.last_scan_time = None
            self.data_processor.scan_count = 0

    def reset_layout(self):
        """重置布局"""
        for plot in self.plots.values():
            plot.enable_auto_range(True)
