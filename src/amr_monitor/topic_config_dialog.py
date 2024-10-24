from PyQt5.QtWidgets import *
from PyQt5.QtCore import *
import yaml
import os
import rospy


class TopicConfigDialog(QDialog):
    def __init__(self, parent=None):
        super().__init__(parent)
        self.setWindowTitle("话题配置")
        self.setup_ui()
        self.load_config()

    def setup_ui(self):
        layout = QVBoxLayout(self)

        # 创建表单布局
        form = QFormLayout()

        # 创建话题输入框
        self.topic_inputs = {}
        default_topics = {
            'imu': '/imu/data',
            'odom': '/odom',
            'scan': '/scan',
            'cmd_vel': '/cmd_vel'
        }

        for name, default in default_topics.items():
            self.topic_inputs[name] = QLineEdit(default)
            form.addRow(f"{name}话题:", self.topic_inputs[name])

        layout.addLayout(form)

        # 添加话题验证
        validate_group = QHBoxLayout()
        validate_btn = QPushButton("验证话题")
        validate_btn.clicked.connect(self.validate_topics)
        validate_group.addWidget(validate_btn)

        self.status_label = QLabel()
        validate_group.addWidget(self.status_label)
        layout.addLayout(validate_group)

        # 按钮组
        btn_group = QHBoxLayout()
        save_btn = QPushButton("保存")
        cancel_btn = QPushButton("取消")

        save_btn.clicked.connect(self.save_config)
        cancel_btn.clicked.connect(self.reject)

        btn_group.addWidget(save_btn)
        btn_group.addWidget(cancel_btn)
        layout.addLayout(btn_group)

    def load_config(self):
        """加载现有配置"""
        config_file = os.path.expanduser('~/.amr_monitor/topics.yaml')
        if os.path.exists(config_file):
            with open(config_file, 'r') as f:
                config = yaml.safe_load(f)
                if config and 'topics' in config:
                    for name, topic in config['topics'].items():
                        if name in self.topic_inputs:
                            self.topic_inputs[name].setText(topic)

    def save_config(self):
        """保存话题配置"""
        config = {
            'topics': {
                name: input.text()
                for name, input in self.topic_inputs.items()
            }
        }

        config_dir = os.path.expanduser('~/.amr_monitor')
        os.makedirs(config_dir, exist_ok=True)

        with open(os.path.join(config_dir, 'topics.yaml'), 'w') as f:
            yaml.dump(config, f, default_flow_style=False)

        self.accept()

    def validate_topics(self):
        """验证话题是否存在"""
        valid_topics = []
        invalid_topics = []

        try:
            topics = rospy.get_published_topics()
            topic_names = [t[0] for t in topics]

            for name, input in self.topic_inputs.items():
                topic = input.text()
                if topic in topic_names:
                    valid_topics.append(name)
                else:
                    invalid_topics.append(name)

            # 更新状态显示
            if invalid_topics:
                self.status_label.setText(
                    f"无效话题: {', '.join(invalid_topics)}")
                self.status_label.setStyleSheet("color: red")
            else:
                self.status_label.setText("所有话题有效")
                self.status_label.setStyleSheet("color: green")

        except Exception as e:
            self.status_label.setText(f"验证失败: {str(e)}")
            self.status_label.setStyleSheet("color: red")
