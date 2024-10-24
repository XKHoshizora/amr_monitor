#!/usr/bin/env python3

import unittest
import sys
import rospy
import rostest
from PyQt5.QtWidgets import QApplication
from amr_monitor.main_window import MainWindow
from amr_monitor.plot_widget import PlotWidget
from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry
import numpy as np


class TestAMRMonitor(unittest.TestCase):
    def setUp(self):
        self.app = QApplication(sys.argv)
        rospy.init_node('test_amr_monitor')

    def test_plot_widget(self):
        """测试绘图组件"""
        widget = PlotWidget()

        # 测试数据缓冲区初始化
        self.assertEqual(len(widget.time_data), widget.buffer_size)
        self.assertEqual(len(widget.imu_data['angular_velocity']['x']),
                         widget.buffer_size)

        # 测试IMU回调
        imu_msg = Imu()
        imu_msg.angular_velocity.x = 1.0
        imu_msg.angular_velocity.y = 2.0
        imu_msg.angular_velocity.z = 3.0
        widget.imu_callback(imu_msg)

        self.assertEqual(widget.imu_data['angular_velocity']['x'][-1], 1.0)
        self.assertEqual(widget.imu_data['angular_velocity']['y'][-1], 2.0)
        self.assertEqual(widget.imu_data['angular_velocity']['z'][-1], 3.0)

        # 测试里程计回调
        odom_msg = Odometry()
        odom_msg.twist.twist.linear.x = 0.5
        odom_msg.twist.twist.angular.z = 0.1
        widget.odom_callback(odom_msg)

        self.assertEqual(widget.odom_data['velocity']['linear'][-1], 0.5)
        self.assertEqual(widget.odom_data['velocity']['angular'][-1], 0.1)

    def test_parameter_widget(self):
        """测试参数控制组件"""
        pass  # TODO: 实现参数控制测试

    def test_data_recording(self):
        """测试数据记录功能"""
        pass  # TODO: 实现数据记录测试


if __name__ == '__main__':
    rostest.rosrun('amr_monitor', 'test_amr_monitor', TestAMRMonitor)
