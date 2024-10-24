from PyQt5.QtWidgets import *
from PyQt5.QtCore import *
import rospy
from dynamic_reconfigure.client import Client


class ParameterWidget(QWidget):
    def __init__(self, parent=None):
        super().__init__(parent)
        self.setup_ui()
        self.setup_clients()

    def setup_ui(self):
        layout = QVBoxLayout(self)

        # AMCL参数组
        amcl_group = QGroupBox("AMCL参数")
        amcl_layout = QFormLayout()
        self.amcl_params = {
            'min_particles': self.create_spinbox(1, 10000, 100),
            'max_particles': self.create_spinbox(1, 50000, 5000),
            'kld_err': self.create_double_spinbox(0.0, 1.0, 0.01, 0.05),
            'update_min_d': self.create_double_spinbox(0.0, 5.0, 0.1, 0.2),
            'update_min_a': self.create_double_spinbox(0.0, 6.28, 0.1, 0.5)
        }
        for name, widget in self.amcl_params.items():
            widget.valueChanged.connect(
                lambda v, n=name: self.update_amcl_param(n, v))
            amcl_layout.addRow(name, widget)
        amcl_group.setLayout(amcl_layout)
        layout.addWidget(amcl_group)

        # DWA参数组
        dwa_group = QGroupBox("DWA参数")
        dwa_layout = QFormLayout()
        self.dwa_params = {
            'max_vel_x': self.create_double_spinbox(0.0, 2.0, 0.1, 0.5),
            'min_vel_x': self.create_double_spinbox(-2.0, 0.0, 0.1, -0.5),
            'max_vel_theta': self.create_double_spinbox(0.0, 3.14, 0.1, 1.0),
            'acc_lim_x': self.create_double_spinbox(0.0, 5.0, 0.1, 2.5),
            'acc_lim_theta': self.create_double_spinbox(0.0, 5.0, 0.1, 3.2)
        }
        for name, widget in self.dwa_params.items():
            widget.valueChanged.connect(
                lambda v, n=name: self.update_dwa_param(n, v))
            dwa_layout.addRow(name, widget)
        dwa_group.setLayout(dwa_layout)
        layout.addWidget(dwa_group)

        # 代价地图参数组
        costmap_group = QGroupBox("代价地图参数")
        costmap_layout = QFormLayout()
        self.costmap_params = {
            'inflation_radius': self.create_double_spinbox(0.0, 2.0, 0.05, 0.55),
            'cost_scaling_factor': self.create_double_spinbox(0.0, 100.0, 1.0, 10.0),
            'obstacle_range': self.create_double_spinbox(0.0, 10.0, 0.1, 2.5),
            'raytrace_range': self.create_double_spinbox(0.0, 20.0, 0.1, 3.0)
        }
        for name, widget in self.costmap_params.items():
            widget.valueChanged.connect(
                lambda v, n=name: self.update_costmap_param(n, v))
            costmap_layout.addRow(name, widget)
        costmap_group.setLayout(costmap_layout)
        layout.addWidget(costmap_group)

        # 添加刷新按钮
        refresh_btn = QPushButton("刷新参数")
        refresh_btn.clicked.connect(self.refresh_parameters)
        layout.addWidget(refresh_btn)

    def create_spinbox(self, min_val, max_val, default_val):
        """创建整数型参数输入框"""
        spinbox = QSpinBox()
        spinbox.setRange(min_val, max_val)
        spinbox.setValue(default_val)
        return spinbox

    def create_double_spinbox(self, min_val, max_val, step, default_val):
        """创建浮点型参数输入框"""
        spinbox = QDoubleSpinBox()
        spinbox.setRange(min_val, max_val)
        spinbox.setSingleStep(step)
        spinbox.setValue(default_val)
        return spinbox

    def setup_clients(self):
        """设置动态参数客户端"""
        try:
            # 使用try-except分别处理每个客户端的连接
            try:
                self.amcl_client = Client("/amcl", timeout=2.0)
                rospy.loginfo("已连接到AMCL参数服务器")
            except Exception as e:
                rospy.logwarn(f"无法连接到AMCL参数服务器: {e}")
                self.amcl_client = None

            try:
                self.dwa_client = Client("/move_base/DWAPlannerROS", timeout=2.0)
                rospy.loginfo("已连接到DWA参数服务器")
            except Exception as e:
                rospy.logwarn(f"无法连接到DWA参数服务器: {e}")
                self.dwa_client = None

            try:
                self.costmap_client = Client("/move_base/global_costmap/inflation_layer", timeout=2.0)
                rospy.loginfo("已连接到代价地图参数服务器")
            except Exception as e:
                rospy.logwarn(f"无法连接到代价地图参数服务器: {e}")
                self.costmap_client = None

            # 初始化参数值
            self.refresh_parameters()

        except Exception as e:
            rospy.logwarn(f"参数服务器初始化失败: {e}")

    def refresh_parameters(self):
        """刷新所有参数值"""
        if hasattr(self, 'amcl_client') and self.amcl_client:
            try:
                config = self.amcl_client.get_configuration()
                for param_name, widget in self.amcl_params.items():
                    if param_name in config:
                        widget.setValue(config[param_name])
            except Exception as e:
                rospy.logwarn(f"刷新AMCL参数失败: {e}")

        if hasattr(self, 'dwa_client') and self.dwa_client:
            try:
                config = self.dwa_client.get_configuration()
                for param_name, widget in self.dwa_params.items():
                    if param_name in config:
                        widget.setValue(config[param_name])
            except Exception as e:
                rospy.logwarn(f"刷新DWA参数失败: {e}")

        if hasattr(self, 'costmap_client') and self.costmap_client:
            try:
                config = self.costmap_client.get_configuration()
                for param_name, widget in self.costmap_params.items():
                    if param_name in config:
                        widget.setValue(config[param_name])
            except Exception as e:
                rospy.logwarn(f"刷新代价地图参数失败: {e}")

    def update_amcl_param(self, name, value):
        """更新AMCL参数"""
        if hasattr(self, 'amcl_client'):
            try:
                self.amcl_client.update_configuration({name: value})
            except rospy.ROSException as e:
                rospy.logwarn(f"更新AMCL参数失败: {e}")

    def update_dwa_param(self, name, value):
        """更新DWA参数"""
        if hasattr(self, 'dwa_client'):
            try:
                self.dwa_client.update_configuration({name: value})
            except rospy.ROSException as e:
                rospy.logwarn(f"更新DWA参数失败: {e}")

    def update_costmap_param(self, name, value):
        """更新代价地图参数"""
        if hasattr(self, 'costmap_client'):
            try:
                self.costmap_client.update_configuration({name: value})
            except rospy.ROSException as e:
                rospy.logwarn(f"更新代价地图参数失败: {e}")
