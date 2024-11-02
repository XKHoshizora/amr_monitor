import unittest
from unittest.mock import Mock
import numpy as np
from amr_monitor.monitors.base_monitor import DataProcessor
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Vector3, Quaternion


class TestDataProcessor(unittest.TestCase):
    def setUp(self):
        self.config = {
            'buffer_size': 100
        }
        self.processor = DataProcessor(self.config['buffer_size'])

    def test_buffer_size(self):
        """测试缓冲区大小限制"""
        # 添加超过buffer_size的数据
        for i in range(150):
            self.processor.update({'value': i})

        # 验证只保留最新的100条数据
        data = self.processor.get_buffer_data()
        self.assertEqual(len(data), 100)
        self.assertEqual(data[-1]['value'], 149)

    def test_statistics(self):
        """测试统计信息计算"""
        test_data = [{'value': i} for i in range(10)]
        for data in test_data:
            self.processor.update(data)

        stats = self.processor.get_statistics()
        self.assertIn('value', stats)
        self.assertEqual(stats['value']['min'], 0)
        self.assertEqual(stats['value']['max'], 9)
        self.assertEqual(stats['value']['count'], 10)

    def test_clear(self):
        """测试清空数据"""
        self.processor.update({'value': 1})
        self.processor.clear()

        self.assertEqual(len(self.processor.get_buffer_data()), 0)
        self.assertEqual(len(self.processor.get_statistics()), 0)


class TestIMUDataProcessor(unittest.TestCase):
    def setUp(self):
        from amr_monitor.monitors.imu_monitor import IMUDataProcessor
        self.config = {'buffer_size': 100}
        self.processor = IMUDataProcessor(self.config)

    def test_imu_processing(self):
        """测试IMU数据处理"""
        # 创建模拟的IMU消息
        msg = Imu()
        msg.angular_velocity = Vector3(x=1.0, y=2.0, z=3.0)
        msg.linear_acceleration = Vector3(x=0.1, y=0.2, z=0.3)
        msg.orientation = Quaternion(x=0, y=0, z=0, w=1)

        result = self.processor.process(msg)

        # 验证处理结果
        self.assertIn('angular_velocity', result)
        self.assertIn('linear_acceleration', result)
        self.assertIn('orientation', result)

        ang_vel = result['angular_velocity']
        self.assertEqual(ang_vel['x'], 1.0)
        self.assertEqual(ang_vel['y'], 2.0)
        self.assertEqual(ang_vel['z'], 3.0)

    def test_quaternion_conversion(self):
        """测试四元数转欧拉角"""
        msg = Imu()
        # 设置90度偏航角的四元数
        msg.orientation = Quaternion(x=0, y=0, z=0.7071, w=0.7071)
        msg.angular_velocity = Vector3()
        msg.linear_acceleration = Vector3()

        result = self.processor.process(msg)

        # 验证欧拉角转换
        orientation = result['orientation']
        self.assertAlmostEqual(orientation['yaw'], 90.0, places=1)
        self.assertAlmostEqual(orientation['roll'], 0.0, places=1)
        self.assertAlmostEqual(orientation['pitch'], 0.0, places=1)


if __name__ == '__main__':
    unittest.main()
