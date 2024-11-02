import unittest
from unittest.mock import Mock, patch
from PySide6.QtCore import Qt
from PySide6.QtTest import QTest
from amr_monitor.monitors.base_monitor import BaseMonitor
from amr_monitor.core.event_bus import EventBus


class TestBaseMonitor(unittest.TestCase):
    def setUp(self):
        self.config = {
            'update_interval': 0.1,
            'buffer_size': 100
        }
        self.event_bus = EventBus()
        self.monitor = BaseMonitor(self.config, self.event_bus)

    def test_update_interval_adjustment(self):
        """测试更新频率自适应调整"""
        # 模拟高CPU使用率
        self.monitor._adjust_update_interval(90)
        self.assertGreater(
            self.monitor.current_update_interval,
            self.config['update_interval']
        )

        # 模拟低CPU使用率
        self.monitor._adjust_update_interval(20)
        self.assertLess(
            self.monitor.current_update_interval,
            self.monitor.max_update_interval
        )

    @patch('amr_monitor.monitors.base_monitor.BasePerformanceMonitor')
    def test_performance_monitoring(self, mock_perf):
        """测试性能监控"""
        mock_perf.return_value.get_system_metrics.return_value = {
            'cpu_percent': 50
        }

        self.monitor.update_displays()
        mock_perf.return_value.get_system_metrics.assert_called_once()

    def test_error_handling(self):
        """测试错误处理"""
        test_error = "Test error"
        self.monitor.handle_error(test_error)

        # 验证状态更新
        self.assertEqual(
            self.monitor.status_widget.status_label.text(),
            f"Error: {test_error}"
        )


class TestIMUMonitor(unittest.TestCase):
    def setUp(self):
        self.config = {
            'update_interval': 0.1,
            'buffer_size': 100,
            'topic': '/test/imu'
        }
        self.event_bus = EventBus()

        with patch('rospy.Subscriber'):
            from amr_monitor.monitors.imu_monitor import IMUMonitor
            self.monitor = IMUMonitor(self.config, self.event_bus)

    def test_imu_callback(self):
        """测试IMU回调处理"""
        from sensor_msgs.msg import Imu
        from geometry_msgs.msg import Vector3, Quaternion

        # 创建测试消息
        msg = Imu()
        msg.angular_velocity = Vector3(x=1.0, y=2.0, z=3.0)
        msg.linear_acceleration = Vector3(x=0.1, y=0.2, z=0.3)
        msg.orientation = Quaternion(x=0, y=0, z=0, w=1)

        # 调用回调
        self.monitor.imu_callback(msg)

        # 验证数据处理
        data = self.monitor.data_processor.get_data()
        self.assertIn('angular_velocity', data)
        self.assertIn('linear_acceleration', data)
        self.assertIn('orientation', data)

    def test_export_data(self):
        """测试数据导出"""
        import tempfile
        from pathlib import Path

        # 添加一些测试数据
        test_data = {
            'angular_velocity': {'x': 1.0, 'y': 2.0, 'z': 3.0},
            'linear_acceleration': {'x': 0.1, 'y': 0.2, 'z': 0.3},
            'orientation': {'roll': 0, 'pitch': 0, 'yaw': 0}
        }
        self.monitor.data_processor.update(test_data)

        # 导出数据
        with tempfile.TemporaryDirectory() as tmp_dir:
            filename = Path(tmp_dir) / 'test_export.csv'
            success = self.monitor.export_data(str(filename))

            self.assertTrue(success)
            self.assertTrue(filename.exists())

            # 验证导出文件内容
            import pandas as pd
            df = pd.read_csv(filename)
            self.assertGreater(len(df), 0)


if __name__ == '__main__':
    unittest.main()
