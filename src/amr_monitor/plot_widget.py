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
        self.is_recording = False  # 添加记录状态标志

        # 初始化plots和curves字典
        self.plots = {}
        self.curves = {}

        # 然后再调用其他设置
        self.setup_ui()
        self.setup_data_buffers()
        self.setup_subscribers()

    def setup_ui(self):
        main_layout = QVBoxLayout(self)

        # 创建标签页组件
        self.tab_widget = QTabWidget()
        self.tab_widget.setTabsClosable(True)
        self.tab_widget.setMovable(True)
        self.tab_widget.tabCloseRequested.connect(self.handle_tab_close)
        main_layout.addWidget(self.tab_widget)

        # IMU数据标签页
        imu_widget = QWidget()
        imu_layout = QVBoxLayout(imu_widget)

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

        # 添加IMU标签页
        self.tab_widget.addTab(imu_widget, "IMU数据")

        # 里程计数据标签页
        odom_widget = QWidget()
        odom_layout = QVBoxLayout(odom_widget)

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

        # 添加里程计标签页
        self.tab_widget.addTab(odom_widget, "里程计数据")

        # 激光雷达数据标签页
        scan_widget = QWidget()
        scan_layout = QVBoxLayout(scan_widget)

        scan_plot = pg.PlotWidget(title='激光扫描')
        scan_plot.setBackground('w')
        scan_plot.showGrid(x=True, y=True)
        scan_plot.setAspectLocked()
        scan_plot.setXRange(-10, 10)
        scan_plot.setYRange(-10, 10)
        self.plots['scan'] = scan_plot
        self.curves['scan'] = scan_plot.plot(
            pen=None,
            symbol='o',
            symbolSize=2,
            symbolBrush='b'
        )
        scan_layout.addWidget(scan_plot)

        # 添加激光雷达标签页
        self.tab_widget.addTab(scan_widget, "激光雷达数据")

        # 添加可拖拽功能按钮
        button_layout = QHBoxLayout()
        detach_button = QPushButton("分离当前标签页")
        detach_button.clicked.connect(self.detach_current_tab)
        button_layout.addWidget(detach_button)
        button_layout.addStretch()
        main_layout.addLayout(button_layout)

        # 存储分离的窗口
        self.detached_windows = {}

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
        if not self.is_recording:
            return  # 如果没有在记录，就不更新图表

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

    def set_recording(self, is_recording):
        """设置记录状态"""
        self.is_recording = is_recording

    def handle_tab_close(self, index):
        """处理标签页关闭事件"""
        tab = self.tab_widget.widget(index)
        if tab is not None:
            title = self.tab_widget.tabText(index)
            self.tab_widget.removeTab(index)
            tab.hide()  # 隐藏而不是删除
            # 添加到主窗口的视图菜单中
            main_window = self.parent()
            if main_window and hasattr(main_window, 'add_to_view_menu'):
                main_window.add_to_view_menu(tab, title)

    def detach_current_tab(self):
        """将当前标签页分离到新窗口"""
        current_index = self.tab_widget.currentIndex()
        if current_index < 0:
            return

        # 获取当前标签页内容和标题
        tab = self.tab_widget.widget(current_index)
        title = self.tab_widget.tabText(current_index)

        # 创建新窗口
        new_window = QWidget()
        new_window.setAttribute(Qt.WA_DeleteOnClose, False)
        new_window.setWindowTitle(title)
        new_window.resize(600, 400)  # 设置合适的大小

        # 创建布局
        layout = QVBoxLayout(new_window)
        layout.setContentsMargins(10, 10, 10, 10)  # 设置边距

        # 直接将tab的内容复制到新窗口
        old_layout = tab.layout()
        if old_layout:
            # 复制所有小部件到新窗口
            while old_layout.count():
                item = old_layout.takeAt(0)
                if item.widget():
                    layout.addWidget(item.widget())

        # 添加重新附加按钮
        attach_btn = QPushButton("重新附加到主窗口")
        attach_btn.clicked.connect(lambda: self.attach_tab(tab, title, new_window))
        layout.addWidget(attach_btn)

        # 存储窗口引用
        self.detached_windows[title] = new_window

        # 从标签页中移除
        self.tab_widget.removeTab(current_index)

        # 显示新窗口
        new_window.show()
        new_window.raise_()

    def attach_tab(self, tab, title, window):
        """将标签页重新附加到主窗口"""
        if title in self.detached_windows:
            # 获取窗口中的所有小部件（除了最后的按钮）
            window_layout = window.layout()
            widgets = []
            for i in range(window_layout.count() - 1):  # -1 排除按钮
                widget = window_layout.itemAt(i).widget()
                if widget:
                    widgets.append(widget)

            # 清理tab的布局
            if tab.layout():
                QWidget().setLayout(tab.layout())  # 清理旧布局

            # 创建新布局
            new_layout = QVBoxLayout(tab)

            # 将小部件添加回tab
            for widget in widgets:
                widget.setParent(None)  # 从旧窗口移除
                new_layout.addWidget(widget)

            # 重新添加到标签页
            self.tab_widget.addTab(tab, title)

            # 关闭独立窗口
            window.close()
            del self.detached_windows[title]

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
            # 角速度数据
            angular_key = f'imu_angular_{axis}'
            if angular_key in data:
                self.imu_data['angular_velocity'][axis] = np.roll(
                    self.imu_data['angular_velocity'][axis], -1)
                self.imu_data['angular_velocity'][axis][-1] = data[angular_key]

            # 线加速度数据
            linear_key = f'imu_linear_{axis}'
            if linear_key in data:
                self.imu_data['linear_acceleration'][axis] = np.roll(
                    self.imu_data['linear_acceleration'][axis], -1)
                self.imu_data['linear_acceleration'][axis][-1] = data[linear_key]

        # 更新里程计数据
        for axis in ['x', 'y']:
            pos_key = f'odom_pos_{axis}'
            if pos_key in data:
                self.odom_data['position'][axis] = np.roll(
                    self.odom_data['position'][axis], -1)
                self.odom_data['position'][axis][-1] = data[pos_key]

        if 'odom_vel_linear' in data:
            self.odom_data['velocity']['linear'] = np.roll(
                self.odom_data['velocity']['linear'], -1)
            self.odom_data['velocity']['linear'][-1] = data['odom_vel_linear']

        if 'odom_vel_angular' in data:
            self.odom_data['velocity']['angular'] = np.roll(
                self.odom_data['velocity']['angular'], -1)
            self.odom_data['velocity']['angular'][-1] = data['odom_vel_angular']

        # 更新图表
        self.update_plots()
