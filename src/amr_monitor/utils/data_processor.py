import numpy as np
from collections import deque
from threading import Lock
import rospy


class DataBuffer:
    """线程安全的数据缓冲区"""

    def __init__(self, size=1000):
        self.size = size
        self.buffer = np.zeros(size)
        self.lock = Lock()
        self.index = 0

    def add(self, value):
        with self.lock:
            self.buffer[self.index] = value
            self.index = (self.index + 1) % self.size

    def get_data(self):
        with self.lock:
            return np.roll(self.buffer, -self.index)


class DataProcessor:
    """数据处理器"""

    def __init__(self, buffer_size=1000):
        self.buffer_size = buffer_size
        self.data_buffers = {}

    def initialize_buffer(self, name):
        """初始化数据缓冲区"""
        if name not in self.data_buffers:
            self.data_buffers[name] = DataBuffer(self.buffer_size)

    def add_data(self, name, value):
        """添加数据到缓冲区"""
        self.initialize_buffer(name)
        self.data_buffers[name].add(value)

    def get_data(self, name):
        """获取数据"""
        if name in self.data_buffers:
            return self.data_buffers[name].get_data()
        return None

    def process_imu_data(self, msg):
        """处理IMU数据"""
        # 处理角速度
        self.add_data('angular_velocity_x', msg.angular_velocity.x)
        self.add_data('angular_velocity_y', msg.angular_velocity.y)
        self.add_data('angular_velocity_z', msg.angular_velocity.z)

        # 处理线加速度
        self.add_data('linear_acceleration_x', msg.linear_acceleration.x)
        self.add_data('linear_acceleration_y', msg.linear_acceleration.y)
        self.add_data('linear_acceleration_z', msg.linear_acceleration.z)

    def process_odom_data(self, msg):
        """处理里程计数据"""
        # 处理位置
        self.add_data('position_x', msg.pose.pose.position.x)
        self.add_data('position_y', msg.pose.pose.position.y)

        # 处理速度
        self.add_data('velocity_linear', msg.twist.twist.linear.x)
        self.add_data('velocity_angular', msg.twist.twist.angular.z)
