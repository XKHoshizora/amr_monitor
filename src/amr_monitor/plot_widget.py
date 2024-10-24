import pyqtgraph as pg
import numpy as np
from PyQt5.QtWidgets import *
from PyQt5.QtCore import *
from PyQt5.QtGui import *
import rospy
from sensor_msgs.msg import Imu, LaserScan
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist


class PlotWidget(QWidget):
    def __init__(self, parent=None):
        super().__init__(parent)
        self.setup_ui()
        self.setup_data_buffers()
        self.setup_subscribers()

    def setup_ui(self):
        layout = QVBoxLayout(self)

        # 创建绘图容器
        self.plots = {}
        self.curves = {}

        # IMU数据图表
        imu_container = QGroupBox("IMU数据")
        imu_layout = QVBoxLayout()

        # 角速度图表
        angular_plot = pg.PlotWidget(title='角速度')
        angular_plot.setBackground('w')
        angular_plot.showGrid(x=True, y=True)
        angular_plot.setLabel('left', 'rad/s')
        angular_plot.setLabel('bottom', 'Time (s)')
        self.plots['angular'] = angular_plot
        self.curves['angular_x'] = angular_plot.plot(pen='r', name='x轴')
        self.curves['angular_y'] = angular_plot.plot(pen='g', name='y轴')
        self.curves['angular_z'] = angular_plot.plot(pen='b', name='z轴')
        imu_layout.addWidget(angular_plot)

        # 线加速度图表
        linear_plot = pg.PlotWidget(title='线加速度')
        linear_plot.setBackground('w')
        linear_plot.showGrid(x=True, y=True)
        linear_plot.setLabel('left', 'm/s²')
        linear_plot.setLabel('bottom', 'Time (s)')
        self.plots['linear'] = linear_plot
        self.curves['linear_x'] = linear_plot.plot(pen='r', name='x轴')
        self.curves['linear_y'] = linear_plot.plot(pen='g', name='y轴')
        self.curves['linear_z'] = linear_plot.plot(pen='b', name='z轴')
        imu_layout.addWidget(linear_plot)

        imu_container.setLayout(imu_layout)
        layout.addWidget(imu_container)

        # 里程计数据图表
        odom_container = QGroupBox("里程计数据")
        odom_layout = QVBoxLayout()

        velocity_plot = pg.PlotWidget(title='速度')
        velocity_plot.setBackground('w')
        velocity_plot.showGrid(x=True, y=True)
        velocity_plot.setLabel('left', 'm/s')
        velocity_plot.setLabel('bottom', 'Time (s)')
        self.plots['velocity'] = velocity_plot
        self.curves['vel_linear'] = velocity_plot.plot(pen='r', name='线速度')
        self.curves['vel_angular'] = velocity_plot.plot(pen='b', name='角速度')
        odom_layout.addWidget(velocity_plot)

        position_plot = pg.PlotWidget(title='位置')
        position_plot.setBackground('w')
        position_plot.showGrid(x=True, y=True)
        position_plot.setLabel('left', 'm')
        position_plot.setLabel('bottom', 'Time (s)')
        self.plots['position'] = position_plot
        self.curves['pos_x'] = position_plot.plot(pen='r', name='x')
        self.curves['pos_y'] = position_plot.plot(pen='g', name='y')
        odom_layout.addWidget(position_plot)

        odom_container.setLayout(odom_layout)
        layout.addWidget(odom_container)

        # 激光雷达数据视图
        scan_container = QGroupBox("激光雷达数据")
        scan_layout = QVBoxLayout()

        scan_plot = pg.PlotWidget(title='激光扫描')
        scan_plot.setBackground('w')
        scan_plot.showGrid(x=True, y=True)
        scan_plot.setAspectLocked()
        scan_plot.setXRange(-10, 10)
        scan_plot.setYRange(-10, 10)
        self.plots['scan'] = scan_plot
        self.curves['scan'] = scan_plot.plot(pen=None, symbol='o',
                                             symbolSize=2, symbolBrush='b')
        scan_layout.addWidget(scan_plot)

        scan_container.setLayout(scan_layout)
        layout.addWidget(scan_container)

    def setup_data_buffers(self):
        """初始化数据缓冲区"""
        self.buffer_size = 500
        self.time_data = np.zeros(self.buffer_size)

        # IMU数据缓冲
        self.imu_data = {
            'angular_velocity': {axis: np.zeros(self.buffer_size) for axis in ['x', 'y', 'z']},
            'linear_acceleration': {axis: np.zeros(self.buffer_size) for axis in ['x', 'y', 'z']}
        }

        # 里程计数据缓冲
        self.odom_data = {
            'position': {axis: np.zeros(self.buffer_size) for axis in ['x', 'y']},
            'velocity': {
                'linear': np.zeros(self.buffer_size),
                'angular': np.zeros(self.buffer_size)
            }
        }

        # 激光数据缓冲
        self.scan_data = {'x': [], 'y': []}

    def setup_subscribers(self):
        """设置ROS话题订阅"""
        rospy.Subscriber('/imu/data', Imu, self.imu_callback)
        rospy.Subscriber('/odom', Odometry, self.odom_callback)
        rospy.Subscriber('/scan', LaserScan, self.scan_callback)

    def imu_callback(self, msg):
        """处理IMU数据"""
        for data_type, msg_field in [('angular_velocity', msg.angular_velocity),
                                     ('linear_acceleration', msg.linear_acceleration)]:
            for axis in ['x', 'y', 'z']:
                data = self.imu_data[data_type][axis]
                data = np.roll(data, -1)
                data[-1] = getattr(msg_field, axis)
                self.imu_data[data_type][axis] = data

    def odom_callback(self, msg):
        """处理里程计数据"""
        for axis in ['x', 'y']:
            self.odom_data['position'][axis] = np.roll(
                self.odom_data['position'][axis], -1)
            self.odom_data['position'][axis][-1] = getattr(
                msg.pose.pose.position, axis)

        self.odom_data['velocity']['linear'] = np.roll(
            self.odom_data['velocity']['linear'], -1)
        self.odom_data['velocity']['linear'][-1] = msg.twist.twist.linear.x

        self.odom_data['velocity']['angular'] = np.roll(
            self.odom_data['velocity']['angular'], -1)
        self.odom_data['velocity']['angular'][-1] = msg.twist.twist.angular.z

    def scan_callback(self, msg):
        """处理激光雷达数据"""
        angles = np.arange(msg.angle_min, msg.angle_max +
                           msg.angle_increment, msg.angle_increment)
        ranges = np.array(msg.ranges)

        # 过滤无效数据
        valid_idx = np.isfinite(ranges) & (
            ranges > msg.range_min) & (ranges < msg.range_max)
        ranges = ranges[valid_idx]
        angles = angles[valid_idx]

        # 计算x,y坐标
        self.scan_data['x'] = ranges * np.cos(angles)
        self.scan_data['y'] = ranges * np.sin(angles)

    def update_plots(self):
        """更新所有图表"""
        current_time = rospy.Time.now().to_sec()
        self.time_data = np.roll(self.time_data, -1)
        self.time_data[-1] = current_time

        # 更新IMU图表
        for axis in ['x', 'y', 'z']:
            self.curves[f'angular_{axis}'].setData(
                self.time_data,
                self.imu_data['angular_velocity'][axis]
            )
            self.curves[f'linear_{axis}'].setData(
                self.time_data,
                self.imu_data['linear_acceleration'][axis]
            )

        # 更新里程计图表
        self.curves['vel_linear'].setData(
            self.time_data,
            self.odom_data['velocity']['linear']
        )
        self.curves['vel_angular'].setData(
            self.time_data,
            self.odom_data['velocity']['angular']
        )

        for axis in ['x', 'y']:
            self.curves[f'pos_{axis}'].setData(
                self.time_data,
                self.odom_data['position'][axis]
            )

        # 更新激光雷达图表
        if self.scan_data['x'] and self.scan_data['y']:
            self.curves['scan'].setData(
                self.scan_data['x'], self.scan_data['y'])

    def get_current_data(self):
        """获取当前数据，用于记录"""
        return {
            'timestamp': rospy.Time.now().to_sec(),
            'imu_angular_x': self.imu_data['angular_velocity']['x'][-1],
            'imu_angular_y': self.imu_data['angular_velocity']['y'][-1],
            'imu_angular_z': self.imu_data['angular_velocity']['z'][-1],
            'imu_linear_x': self.imu_data['linear_acceleration']['x'][-1],
            'imu_linear_y': self.imu_data['linear_acceleration']['y'][-1],
            'imu_linear_z': self.imu_data['linear_acceleration']['z'][-1],
            'odom_pos_x': self.odom_data['position']['x'][-1],
            'odom_pos_y': self.odom_data['position']['y'][-1],
            'odom_vel_linear': self.odom_data['velocity']['linear'][-1],
            'odom_vel_angular': self.odom_data['velocity']['angular'][-1]
        }

    def update_from_playback(self, data):
        """从回放数据更新图表"""
        # 更新IMU数据
        for axis in ['x', 'y', 'z']:
            # 修复角速度数据更新
            self.imu_data['angular_velocity'][axis] = np.roll(self.imu_data['angular_velocity'][axis], -1)
            self.imu_data['angular_velocity'][axis][-1] = data[f'imu_angular_{axis}']

            # 修复线加速度数据更新
            self.imu_data['linear_acceleration'][axis] = np.roll(self.imu_data['linear_acceleration'][axis], -1)
            self.imu_data['linear_acceleration'][axis][-1] = data[f'imu_linear_{axis}']

        # 更新里程计数据
        for axis in ['x', 'y']:
            self.odom_data['position'][axis] = np.roll(self.odom_data['position'][axis], -1)
            self.odom_data['position'][axis][-1] = data[f'odom_pos_{axis}']

        self.odom_data['velocity']['linear'] = np.roll(self.odom_data['velocity']['linear'], -1)
        self.odom_data['velocity']['linear'][-1] = data['odom_vel_linear']

        self.odom_data['velocity']['angular'] = np.roll(self.odom_data['velocity']['angular'], -1)
        self.odom_data['velocity']['angular'][-1] = data['odom_vel_angular']

        # 更新图表
        self.update_plots()
