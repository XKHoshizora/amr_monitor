# src/amr_monitor/monitors/imu_monitor.py
"""IMU监控器模块"""
import numpy as np
import tf.transformations as tf
from typing import Dict, Any
import rospy
from sensor_msgs.msg import Imu

from amr_monitor.core.logger import Logger
from amr_monitor.core.event_bus import EventBus
from .base_monitor import BaseMonitor, DataProcessor

logger = Logger.get_logger(__name__)

class IMUDataProcessor(DataProcessor):
    """IMU数据处理器"""

    def process(self, msg: Imu) -> Dict[str, Any]:
        """处理IMU数据

        Args:
            msg (Imu): ROS IMU消息

        Returns:
            Dict[str, Any]: 处理后的数据字典
        """
        # 提取角速度数据 (rad/s)
        angular_velocity = {
            'x': msg.angular_velocity.x,
            'y': msg.angular_velocity.y,
            'z': msg.angular_velocity.z
        }

        # 提取线加速度数据 (m/s²)
        linear_acceleration = {
            'x': msg.linear_acceleration.x,
            'y': msg.linear_acceleration.y,
            'z': msg.linear_acceleration.z
        }

        # 转换姿态数据
        quaternion = [
            msg.orientation.x,
            msg.orientation.y,
            msg.orientation.z,
            msg.orientation.w
        ]
        roll, pitch, yaw = tf.euler_from_quaternion(quaternion)

        # 转换为角度
        orientation = {
            'roll': np.degrees(roll),
            'pitch': np.degrees(pitch),
            'yaw': np.degrees(yaw)
        }

        return {
            'angular_velocity': angular_velocity,
            'linear_acceleration': linear_acceleration,
            'orientation': orientation,
            'timestamp': msg.header.stamp.to_sec()
        }

class IMUMonitor(BaseMonitor):
    """IMU监控器"""

    def __init__(self, config: Dict[str, Any], event_bus: EventBus, parent=None):
        super().__init__(config, event_bus, parent)

        # 创建数据处理器
        self.data_processor = IMUDataProcessor(
            buffer_size=config.get('buffer_size', 500)
        )

        # 设置图表
        self.setup_plots()

        # 设置ROS话题订阅
        self.topic_sub = None
        self.setup_subscriber()

    def setup_plots(self):
        """设置图表"""
        # 角速度图表
        self.add_plot(
            'angular_velocity',
            title='Angular Velocity',
            x_label='Time (s)',
            y_label='rad/s'
        )

        # 线加速度图表
        self.add_plot(
            'linear_acceleration',
            title='Linear Acceleration',
            x_label='Time (s)',
            y_label='m/s²'
        )

        # 姿态图表
        self.add_plot(
            'orientation',
            title='Orientation',
            x_label='Time (s)',
            y_label='degrees'
        )

    def setup_subscriber(self):
        """设置话题订阅"""
        if self.topic_sub:
            self.topic_sub.unregister()

        topic = self.topic_combobox.currentText() or self.config.get('topic', '/imu/data')
        try:
            self.topic_sub = rospy.Subscriber(
                topic,
                Imu,
                self.imu_callback,
                queue_size=10
            )
            self.status_label.setText(f"Connected to {topic}")
            logger.info(f"Subscribed to IMU topic: {topic}")
        except Exception as e:
            self.handle_error(f"Failed to subscribe to {topic}: {e}")

    def imu_callback(self, msg: Imu):
        """IMU消息回调处理"""
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
            self.handle_error(f"IMU data processing error: {e}")

    def _update_plots(self, data: Dict[str, Any]):
        """更新图表显示

        Args:
            data (Dict[str, Any]): 处理后的IMU数据
        """
        timestamp = data['timestamp']

        # 更新角速度图
        angular_vel = data['angular_velocity']
        for axis in ['x', 'y', 'z']:
            self.plots['angular_velocity'].update_curve(
                f'angular_vel_{axis}',
                timestamp,
                angular_vel[axis]
            )

        # 更新线加速度图
        linear_acc = data['linear_acceleration']
        for axis in ['x', 'y', 'z']:
            self.plots['linear_acceleration'].update_curve(
                f'linear_acc_{axis}',
                timestamp,
                linear_acc[axis]
            )

        # 更新姿态图
        orientation = data['orientation']
        for axis in ['roll', 'pitch', 'yaw']:
            self.plots['orientation'].update_curve(
                axis,
                timestamp,
                orientation[axis]
            )

    def _update_info(self, data: Dict[str, Any]):
        """更新信息显示

        Args:
            data (Dict[str, Any]): 处理后的IMU数据
        """
        ang_vel = data['angular_velocity']
        lin_acc = data['linear_acceleration']
        orientation = data['orientation']

        info_text = (
            f"Angular Velocity (rad/s): "
            f"x={ang_vel['x']:.2f}, y={ang_vel['y']:.2f}, z={ang_vel['z']:.2f}\n"
            f"Linear Acceleration (m/s²): "
            f"x={lin_acc['x']:.2f}, y={lin_acc['y']:.2f}, z={lin_acc['z']:.2f}\n"
            f"Orientation (deg): "
            f"roll={orientation['roll']:.2f}, pitch={orientation['pitch']:.2f}, "
            f"yaw={orientation['yaw']:.2f}"
        )

        for widget in self.info_frame.findChildren(QLabel):
            widget.setText(info_text)

    def get_status_text(self, data: Dict[str, Any]) -> str:
        """获取状态文本

        Args:
            data (Dict[str, Any]): 处理后的IMU数据

        Returns:
            str: 状态文本
        """
        orientation = data['orientation']
        return (f"Roll: {orientation['roll']:.1f}°, "
                f"Pitch: {orientation['pitch']:.1f}°, "
                f"Yaw: {orientation['yaw']:.1f}°")

    def _on_refresh_clicked(self):
        """刷新按钮点击处理"""
        try:
            # 获取当前可用的IMU话题
            imu_topics = [
                topic for topic, msg_type in rospy.get_published_topics()
                if msg_type == 'sensor_msgs/Imu'
            ]

            # 更新话题列表
            current = self.topic_combobox.currentText()
            self.topic_combobox.clear()
            self.topic_combobox.addItems(imu_topics)

            # 尝试恢复之前选择的话题
            index = self.topic_combobox.findText(current)
            if index >= 0:
                self.topic_combobox.setCurrentIndex(index)

            # 重新设置订阅
            self.setup_subscriber()

        except Exception as e:
            self.handle_error(f"Failed to refresh topics: {e}")

    def reset_layout(self):
        """重置布局"""
        for plot in self.plots.values():
            plot.enable_auto_range(True)