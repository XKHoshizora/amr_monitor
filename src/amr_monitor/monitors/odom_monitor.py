# src/amr_monitor/monitors/odom_monitor.py
"""里程计监控器"""
import numpy as np
import tf.transformations as tf
from typing import Dict, Any
import rospy
from nav_msgs.msg import Odometry

from amr_monitor.core.logger import Logger
from amr_monitor.core.event_bus import EventBus
from .base_monitor import BaseMonitor, DataProcessor

logger = Logger.get_logger(__name__)


class OdomDataProcessor(DataProcessor):
    """里程计数据处理器"""

    def __init__(self, buffer_size: int = 1000):
        super().__init__(buffer_size)
        self.total_distance = 0.0
        self.last_position = None
        self.start_time = None

    def process(self, msg: Odometry) -> Dict[str, Any]:
        """处理里程计数据

        Args:
            msg (Odometry): ROS里程计消息

        Returns:
            Dict[str, Any]: 处理后的数据字典
        """
        # 记录开始时间
        if self.start_time is None:
            self.start_time = msg.header.stamp.to_sec()

        # 提取位置数据
        position = {
            'x': msg.pose.pose.position.x,
            'y': msg.pose.pose.position.y,
            'z': msg.pose.pose.position.z
        }

        # 提取速度数据
        velocity = {
            'linear': {
                'x': msg.twist.twist.linear.x,
                'y': msg.twist.twist.linear.y,
                'z': msg.twist.twist.linear.z
            },
            'angular': {
                'x': msg.twist.twist.angular.x,
                'y': msg.twist.twist.angular.y,
                'z': msg.twist.twist.angular.z
            }
        }

        # 计算方向角
        quaternion = [
            msg.pose.pose.orientation.x,
            msg.pose.pose.orientation.y,
            msg.pose.pose.orientation.z,
            msg.pose.pose.orientation.w
        ]
        roll, pitch, yaw = tf.euler_from_quaternion(quaternion)
        orientation = {
            'roll': np.degrees(roll),
            'pitch': np.degrees(pitch),
            'yaw': np.degrees(yaw)
        }

        # 计算累计移动距离
        if self.last_position is not None:
            dx = position['x'] - self.last_position['x']
            dy = position['y'] - self.last_position['y']
            dz = position['z'] - self.last_position['z']
            step_distance = np.sqrt(dx*dx + dy*dy + dz*dz)
            self.total_distance += step_distance

        self.last_position = position

        # 计算合速度
        linear_speed = np.sqrt(
            velocity['linear']['x']**2 +
            velocity['linear']['y']**2 +
            velocity['linear']['z']**2
        )
        angular_speed = np.sqrt(
            velocity['angular']['x']**2 +
            velocity['angular']['y']**2 +
            velocity['angular']['z']**2
        )

        return {
            'position': position,
            'velocity': velocity,
            'orientation': orientation,
            'linear_speed': linear_speed,
            'angular_speed': angular_speed,
            'total_distance': self.total_distance,
            'timestamp': msg.header.stamp.to_sec(),
            'duration': msg.header.stamp.to_sec() - self.start_time
        }


class OdomMonitor(BaseMonitor):
    """里程计监控器"""

    def __init__(self, config: Dict[str, Any], event_bus: EventBus, parent=None):
        super().__init__(config, event_bus, parent)

        # 创建数据处理器
        self.data_processor = OdomDataProcessor(
            buffer_size=config.get('buffer_size', 1000)
        )

        # 设置图表
        self.setup_plots()

        # 设置ROS话题订阅
        self.topic_sub = None
        self.setup_subscriber()

    def setup_plots(self):
        """设置图表"""
        # 轨迹图
        self.add_plot(
            'trajectory',
            title='Robot Trajectory',
            x_label='X Position (m)',
            y_label='Y Position (m)'
        )
        self.plots['trajectory'].plot.setAspectLocked(True)

        # 线速度图
        self.add_plot(
            'linear_velocity',
            title='Linear Velocity',
            x_label='Time (s)',
            y_label='m/s'
        )

        # 角速度图
        self.add_plot(
            'angular_velocity',
            title='Angular Velocity',
            x_label='Time (s)',
            y_label='rad/s'
        )

    def setup_subscriber(self):
        """设置话题订阅"""
        if self.topic_sub:
            self.topic_sub.unregister()

        topic = self.topic_combobox.currentText() or self.config.get('topic', '/odom')
        try:
            self.topic_sub = rospy.Subscriber(
                topic,
                Odometry,
                self.odom_callback,
                queue_size=10
            )
            self.status_label.setText(f"Connected to {topic}")
            logger.info(f"Subscribed to odometry topic: {topic}")
        except Exception as e:
            self.handle_error(f"Failed to subscribe to {topic}: {e}")

    def odom_callback(self, msg: Odometry):
        """里程计消息回调处理"""
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
            self.handle_error(f"Odometry data processing error: {e}")

    def _update_plots(self, data: Dict[str, Any]):
        """更新图表显示"""
        # 更新轨迹图
        pos = data['position']
        self.plots['trajectory'].update_curve(
            'robot_trajectory',
            pos['x'],
            pos['y']
        )

        # 更新线速度图
        vel = data['velocity']['linear']
        time = data['duration']  # 使用相对时间
        for axis in ['x', 'y', 'z']:
            self.plots['linear_velocity'].update_curve(
                f'linear_vel_{axis}',
                time,
                vel[axis]
            )

        # 更新角速度图
        ang_vel = data['velocity']['angular']
        for axis in ['x', 'y', 'z']:
            self.plots['angular_velocity'].update_curve(
                f'angular_vel_{axis}',
                time,
                ang_vel[axis]
            )

    def _update_info(self, data: Dict[str, Any]):
        """更新信息显示"""
        pos = data['position']
        vel = data['velocity']
        ori = data['orientation']

        info_text = (
            f"Position (m): "
            f"x={pos['x']:.2f}, y={pos['y']:.2f}, z={pos['z']:.2f}\n"
            f"Linear Speed: {data['linear_speed']:.2f} m/s\n"
            f"Angular Speed: {data['angular_speed']:.2f} rad/s\n"
            f"Orientation (deg): "
            f"roll={ori['roll']:.2f}, pitch={ori['pitch']:.2f}, "
            f"yaw={ori['yaw']:.2f}\n"
            f"Total Distance: {data['total_distance']:.2f} m"
        )

        for widget in self.info_frame.findChildren(QLabel):
            widget.setText(info_text)

    def get_status_text(self, data: Dict[str, Any]) -> str:
        """获取状态文本"""
        return (f"Speed: {data['linear_speed']:.2f} m/s, "
                f"Distance: {data['total_distance']:.2f} m")

    def _on_refresh_clicked(self):
        """刷新按钮点击处理"""
        try:
            # 获取当前可用的里程计话题
            odom_topics = [
                topic for topic, msg_type in rospy.get_published_topics()
                if msg_type == 'nav_msgs/Odometry'
            ]

            # 更新话题列表
            current = self.topic_combobox.currentText()
            self.topic_combobox.clear()
            self.topic_combobox.addItems(odom_topics)

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
            self.data_processor.total_distance = 0.0
            self.data_processor.last_position = None
            self.data_processor.start_time = None

    def reset_layout(self):
        """重置布局"""
        for plot in self.plots.values():
            plot.enable_auto_range(True)
