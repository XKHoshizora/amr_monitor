# src/amr_monitor/config_manager.py

import os
import yaml
import json
from PyQt5.QtWidgets import *
from PyQt5.QtCore import *
import rospy
import rospkg

class ConfigManager:
    """配置管理器"""
    def __init__(self):
        # 使用rospkg获取功能包路径
        rospack = rospkg.RosPack()
        self.package_path = rospack.get_path('amr_monitor')
        self.config_dir = os.path.join(self.package_path, 'config')
        self.ensure_config_dir()
        self.current_config = {}
        self.load_default_config()

    def ensure_config_dir(self):
        """确保配置目录存在"""
        try:
            os.makedirs(self.config_dir, exist_ok=True)
        except Exception as e:
            rospy.logerr(f"创建配置目录失败: {str(e)}")
            raise

    def get_config_path(self, filename='config.yaml'):
        """获取配置文件完整路径"""
        return os.path.join(self.config_dir, filename)

    def load_default_config(self):
        """加载默认配置"""
        default_config = {
            'ui': {
                'theme': 'light',
                'window': {
                    'width': 1200,
                    'height': 800
                },
                'refresh_rate': 30
            },
            'topics': {
                'imu': '/imu/data',
                'odom': '/odom',
                'scan': '/scan',
                'cmd_vel': '/cmd_vel'
            },
            'parameters': {
                'amcl': {
                    'min_particles': 100,
                    'max_particles': 5000,
                    'kld_err': 0.05,
                    'update_min_d': 0.2,
                    'update_min_a': 0.5
                },
                'dwa': {
                    'max_vel_x': 0.5,
                    'min_vel_x': -0.5,
                    'max_vel_theta': 1.0,
                    'acc_lim_x': 2.5,
                    'acc_lim_theta': 3.2
                },
                'costmap': {
                    'inflation_radius': 0.55,
                    'cost_scaling_factor': 10.0,
                    'obstacle_range': 2.5,
                    'raytrace_range': 3.0
                }
            },
            'buffer': {
                'size': 1000,
                'auto_clear': True
            },
            'recording': {
                'auto_timestamp': True,
                'save_dir': os.path.join(self.package_path, 'data')
            }
        }

        config_file = self.get_config_path()
        if os.path.exists(config_file):
            try:
                with open(config_file, 'r') as f:
                    user_config = yaml.safe_load(f)
                    if user_config:
                        self.current_config = self.update_dict(default_config, user_config)
                    else:
                        self.current_config = default_config
            except Exception as e:
                rospy.logwarn(f"加载用户配置失败，使用默认配置: {str(e)}")
                self.current_config = default_config
        else:
            self.current_config = default_config
            self.save_config()

    def update_dict(self, d1, d2):
        """递归更新字典"""
        for k, v in d2.items():
            if k in d1 and isinstance(d1[k], dict) and isinstance(v, dict):
                self.update_dict(d1[k], v)
            else:
                d1[k] = v
        return d1

    def save_config(self, config=None):
        """保存配置到文件"""
        if config is not None:
            self.current_config = config

        try:
            config_file = self.get_config_path()
            with open(config_file, 'w') as f:
                yaml.dump(self.current_config, f, default_flow_style=False)
            rospy.loginfo("配置保存成功")
        except Exception as e:
            rospy.logerr(f"保存配置失败: {str(e)}")
            raise

    def get_config(self, section=None):
        """获取配置"""
        if section:
            return self.current_config.get(section, {})
        return self.current_config

    def update_config(self, section, key, value):
        """更新特定配置项"""
        if section in self.current_config:
            if isinstance(self.current_config[section], dict):
                self.current_config[section][key] = value
                self.save_config()
            else:
                rospy.logwarn(f"配置节 {section} 不是字典类型")
        else:
            rospy.logwarn(f"配置节 {section} 不存在")

    def export_config(self, filepath):
        """导出配置到文件"""
        try:
            with open(filepath, 'w') as f:
                yaml.dump(self.current_config, f, default_flow_style=False)
            rospy.loginfo(f"配置导出成功: {filepath}")
        except Exception as e:
            rospy.logerr(f"导出配置失败: {str(e)}")
            raise

    def import_config(self, filepath):
        """从文件导入配置"""
        try:
            with open(filepath, 'r') as f:
                new_config = yaml.safe_load(f)
                if new_config:
                    self.current_config = new_config
                    self.save_config()
                    rospy.loginfo("配置导入成功")
                else:
                    rospy.logwarn("导入的配置文件为空")
        except Exception as e:
            rospy.logerr(f"导入配置失败: {str(e)}")
            raise


class ConfigDialog(QDialog):
    """配置对话框"""

    def __init__(self, config_manager, parent=None):
        super().__init__(parent)
        self.config_manager = config_manager
        self.setup_ui()

    def setup_ui(self):
        self.setWindowTitle("配置管理")
        layout = QVBoxLayout(self)

        # 配置树形视图
        self.tree = QTreeWidget()
        self.tree.setHeaderLabels(["配置项", "值"])
        layout.addWidget(self.tree)

        # 加载配置到树形视图
        self.load_config_to_tree()

        # 按钮组
        btn_group = QHBoxLayout()
        save_btn = QPushButton("保存")
        import_btn = QPushButton("导入")
        export_btn = QPushButton("导出")

        save_btn.clicked.connect(self.save_changes)
        import_btn.clicked.connect(self.import_config)
        export_btn.clicked.connect(self.export_config)

        btn_group.addWidget(save_btn)
        btn_group.addWidget(import_btn)
        btn_group.addWidget(export_btn)
        layout.addLayout(btn_group)

    def load_config_to_tree(self):
        """加载配置到树形视图"""
        self.tree.clear()
        config = self.config_manager.get_config()

        for section, values in config.items():
            section_item = QTreeWidgetItem(self.tree, [section])
            self.add_items(section_item, values)

        self.tree.expandAll()

    def add_items(self, parent, values):
        """递归添加配置项到树形视图"""
        if isinstance(values, dict):
            for key, value in values.items():
                item = QTreeWidgetItem(parent, [key])
                self.add_items(item, value)
        else:
            parent.setText(1, str(values))

    def save_changes(self):
        """保存修改的配置"""
        try:
            # 从树形视图获取修改后的配置
            config = self.get_config_from_tree()
            self.config_manager.save_config(config)
            QMessageBox.information(self, "成功", "配置已保存")
        except Exception as e:
            QMessageBox.critical(self, "错误", f"保存配置失败: {str(e)}")

    def get_config_from_tree(self):
        """从树形视图获取配置"""
        config = {}
        root = self.tree.invisibleRootItem()

        for i in range(root.childCount()):
            section_item = root.child(i)
            section_name = section_item.text(0)
            config[section_name] = self.get_item_value(section_item)

        return config

    def get_item_value(self, item):
        """递归获取配置项值"""
        if item.childCount() == 0:
            # 尝试转换为适当的类型
            value = item.text(1)
            try:
                # 尝试转换为数值
                if '.' in value:
                    return float(value)
                return int(value)
            except ValueError:
                # 如果不是数值，返回字符串
                return value
        else:
            result = {}
            for i in range(item.childCount()):
                child = item.child(i)
                result[child.text(0)] = self.get_item_value(child)
            return result

    def import_config(self):
        """导入配置"""
        filepath, _ = QFileDialog.getOpenFileName(
            self, "导入配置", "", "YAML Files (*.yaml);;All Files (*)")
        if filepath:
            try:
                self.config_manager.import_config(filepath)
                self.load_config_to_tree()
                QMessageBox.information(self, "成功", "配置导入成功")
            except Exception as e:
                QMessageBox.critical(self, "错误", f"导入配置失败: {str(e)}")

    def export_config(self):
        """导出配置"""
        filepath, _ = QFileDialog.getSaveFileName(
            self, "导出配置", "", "YAML Files (*.yaml);;All Files (*)")
        if filepath:
            try:
                self.config_manager.export_config(filepath)
                QMessageBox.information(self, "成功", "配置导出成功")
            except Exception as e:
                QMessageBox.critical(self, "错误", f"导出配置失败: {str(e)}")
