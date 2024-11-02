# src/amr_monitor/monitors/cmd_vel_monitor.py
"""速度监控器"""
import numpy as np
from typing import Dict, Any, Optional
import time
import rospy
from geometry_msgs.msg import Twist

from amr_monitor.core.logger import Logger
from amr_monitor.core.event_bus import EventBus
from .base_monitor import BaseMonitor, DataProcessor

logger = Logger.get_logger(__name__)


class CmdVelDataProcessor(DataProcessor):
    """速度数据处理器"""

    def __init__(self, config: Dict[str, Any], buffer_size: int = 200):
        super().__init__(buffer_size)
        self.max_linear_vel = config.get('max_linear_speed', 1.0)
        self.max_angular_vel = config.get('max_angular_speed', 1.5)
        self.start_time = None
        self.last_cmd_time = None
        self.cmd_count = 0

        # 运动分析
        self.distance = 0.0
        self.rotation = 0.0
        self.last_time = None

    def process(self, msg: Twist) -> Dict[str, Any]:
        """处理速度数据

        Args:
            msg (Twist): ROS速度消息

        Returns:
            Dict[str, Any]: 处理后的数据字典
        """
        current_time = time.time()

        # 记录开始时间
        if self.start_time is None:
            self.start_time = current_time

        # 计算指令频率
        cmd_frequency = 0.0
        if self.last_cmd_time is not None:
            time_diff = current_time - self.last_cmd_time
            if time_diff > 0:
                cmd_frequency = 1.0 / time_diff
        self.last_cmd_time = current_time

        # 累计行驶距离和转向角度
        if self.last_time is not None:
            dt = current_time - self.last_time
            self.distance += abs(msg.linear.x) * dt
            self.rotation += abs(msg.angular.z) * dt
        self.last_time = current_time

        # 提取速度数据
        linear_vel = {
            'x': msg.linear.x,
            'y': msg.linear.y,
            'z': msg.linear.z
        }
        angular_vel = {
            'x': msg.angular.x,
            'y': msg.angular.y,
            'z': msg.angular.z
        }

        # 计算合速度
        linear_speed = np.sqrt(
            msg.linear.x**2 +
            msg.linear.y**2 +
            msg.linear.z**2
        )
        angular_speed = np.sqrt(
            msg.angular.x**2 +
            msg.angular.y**2 +
            msg.angular.z**2
        )

        # 检查速度限制
        status = 'normal'
        if linear_speed > self.max_linear_vel:
            status = 'linear_overspeed'
        elif angular_speed > self.max_angular_vel:
            status = 'angular_overspeed'

        self.cmd_count += 1

        return {
            'linear_velocity': linear_vel,
            'angular_velocity': angular_vel,
            'linear_speed': linear_speed,
            'angular_speed': angular_speed,
            'total_distance': self.distance,
            'total_rotation': self.rotation,
            'stats': {
                'cmd_count': self.cmd_count,
                'frequency': cmd_frequency,
                'status': status,
                'runtime': current_time - self.start_time
            },
            'timestamp': current_time,
            'duration': current_time - self.start_time
        }


class CmdVelMonitor(BaseMonitor):
    """速度监控器"""

    def __init__(self, config: Dict[str, Any], event_bus: EventBus, parent=None):
        super().__init__(config, event_bus, parent)

        # 创建数据处理器
        self.data_processor = CmdVelDataProcessor(
            config,
            buffer_size=config.get('buffer_size', 200)
        )

        # 设置图表
        self.setup_plots()

        # 设置ROS话题订阅
        self.topic_sub = None
        self.setup_subscriber()

    def setup_plots(self):
        """设置图表"""
        # 线速度图表
        self.add_plot(
            'linear_velocity',
            title='Linear Velocity',
            x_label='Time (s)',
            y_label='m/s'
        )

        # 角速度图表
        self.add_plot(
            'angular_velocity',
            title='Angular Velocity',
            x_label='Time (s)',
            y_label='rad/s'
        )

        # 速度限制参考线
        self.plots['linear_velocity'].plot.addLine(
            y=self.data_processor.max_linear_vel,
            pen='r'
        )
        self.plots['angular_velocity'].plot.addLine(
            y=self.data_processor.max_angular_vel,
            pen='r'
        )

    def setup_subscriber(self):
        """设置话题订阅"""
        if self.topic_sub:
            self.topic_sub.unregister()

        topic = self.topic_combobox.currentText() or self.config.get('topic', '/cmd_vel')
        try:
            self.topic_sub = rospy.Subscriber(
                topic,
                Twist,
                self.cmd_vel_callback,
                queue_size=1
            )
            self.status_label.setText(f"Connected to {topic}")
            logger.info(f"Subscribed to velocity topic: {topic}")
        except Exception as e:
            self.handle_error(f"Failed to subscribe to {topic}: {e}")

    def cmd_vel_callback(self, msg: Twist):
        """速度消息回调处理"""
        try:
            # 处理数据
            data = self.data_processor.process(msg)

            # 发布事件
            self.event_bus.publish(Event(
                'monitor_data',
                data,
                self.__class__.__name__
            ))

            # 检查速度限制
            if data['stats']['status'] != 'normal':
                self.handle_overspeed(data)

        except Exception as e:
            self.handle_error(f"Velocity data processing error: {e}")

    def handle_overspeed(self, data: Dict[str, Any]):
        """处理超速情况"""
        status = data['stats']['status']
        if status == 'linear_overspeed':
            msg = (f"Linear velocity ({data['linear_speed']:.2f} m/s) "
                   f"exceeds limit ({self.data_processor.max_linear_vel:.2f} m/s)")
        else:
            msg = (f"Angular velocity ({data['angular_speed']:.2f} rad/s) "
                   f"exceeds limit ({self.data_processor.max_angular_vel:.2f} rad/s)")
        logger.warning(msg)

    def _update_plots(self, data: Dict[str, Any]):
        """更新图表显示"""
        time = data['duration']  # 使用相对时间

        # 更新线速度图
        linear_vel = data['linear_velocity']
        for axis in ['x', 'y', 'z']:
            self.plots['linear_velocity'].update_curve(
                f'linear_vel_{axis}',
                time,
                linear_vel[axis]
            )

        # 更新角速度图
        angular_vel = data['angular_velocity']
        for axis in ['x', 'y', 'z']:
            self.plots['angular_velocity'].update_curve(
                f'angular_vel_{axis}',
                time,
                angular_vel[axis]
            )

    def _update_info(self, data: Dict[str, Any]):
        """更新信息显示"""
        linear = data['linear_velocity']
        angular = data['angular_velocity']
        stats = data['stats']

        info_text = (
            f"Linear Velocity (m/s): "
            f"x={linear['x']:.2f}, y={linear['y']:.2f}, z={linear['z']:.2f}\n"
            f"Angular Velocity (rad/s): "
            f"x={angular['x']:.2f}, y={angular['y']:.2f}, z={angular['z']:.2f}\n"
            f"Speed: {data['linear_speed']:.2f} m/s, "
            f"Turn Rate: {data['angular_speed']:.2f} rad/s\n"
            f"Distance: {data['total_distance']:.2f} m, "
            f"Rotation: {np.degrees(data['total_rotation']):.1f}°\n"
            f"Commands: {stats['cmd_count']}, "
            f"Frequency: {stats['frequency']:.1f} Hz"
        )

        for widget in self.info_frame.findChildren(QLabel):
            widget.setText(info_text)

    def get_status_text(self, data: Dict[str, Any]) -> str:
        """获取状态文本"""
        return (f"Speed: {data['linear_speed']:.2f} m/s, "
                f"Turn: {data['angular_speed']:.2f} rad/s, "
                f"Status: {data['stats']['status']}")

    def _on_refresh_clicked(self):
        """刷新按钮点击处理"""
        try:
            # 获取当前可用的速度话题
            cmd_vel_topics = [
                topic for topic, msg_type in rospy.get_published_topics()
                if msg_type == 'geometry_msgs/Twist'
            ]

            # 更新话题列表
            current = self.topic_combobox.currentText()
            self.topic_combobox.clear()
            self.topic_combobox.addItems(cmd_vel_topics)

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
            self.data_processor.last_cmd_time = None
            self.data_processor.cmd_count = 0
            self.data_processor.distance = 0.0
            self.data_processor.rotation = 0.0
            self.data_processor.last_time = None

    def reset_layout(self):
        """重置布局"""
        for plot in self.plots.values():
            plot.enable_auto_range(True)
