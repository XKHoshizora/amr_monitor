from PyQt5.QtWidgets import *
from PyQt5.QtCore import *
import pandas as pd
import numpy as np
import os


class PlaybackWidget(QWidget):
    """数据回放控件"""
    # 定义信号
    data_updated = pyqtSignal(object)

    def __init__(self, data_dir=None, parent=None):
        super().__init__(parent, Qt.Window)  # 使用Qt.Window标志
        self.setAttribute(Qt.WA_DeleteOnClose, False)  # 不自动删除
        self.setWindowTitle("数据回放")
        self.setMinimumSize(600, 400)
        self.data_dir = data_dir
        self.setup_ui()
        self.data = None
        self.current_index = 0
        self.playing = False

    def setup_ui(self):
        layout = QVBoxLayout(self)
        layout.setContentsMargins(10, 10, 10, 10)  # 设置边距
        layout.setSpacing(10)  # 设置组件间距

        # 文件选择组
        file_group = QGroupBox("文件选择")
        file_layout = QHBoxLayout()
        file_layout.setContentsMargins(5, 5, 5, 5)

        self.file_path = QLineEdit()
        self.file_path.setReadOnly(True)  # 设置为只读
        browse_btn = QPushButton("浏览")
        browse_btn.setMinimumWidth(80)  # 设置最小宽度
        browse_btn.clicked.connect(self.browse_file)

        file_layout.addWidget(self.file_path)
        file_layout.addWidget(browse_btn)
        file_group.setLayout(file_layout)
        layout.addWidget(file_group)

        # 播放控制组
        control_group = QGroupBox("播放控制")
        control_layout = QVBoxLayout()
        control_layout.setContentsMargins(5, 5, 5, 5)

        # 播放控制按钮和滑块
        button_slider_layout = QHBoxLayout()
        self.play_btn = QPushButton("播放")
        self.play_btn.setCheckable(True)
        self.play_btn.setMinimumWidth(80)
        self.play_btn.toggled.connect(self.toggle_playback)

        self.slider = QSlider(Qt.Horizontal)
        self.slider.setMinimumWidth(200)  # 设置最小宽度
        self.slider.valueChanged.connect(self.seek)

        button_slider_layout.addWidget(self.play_btn)
        button_slider_layout.addWidget(self.slider)
        control_layout.addLayout(button_slider_layout)

        # 播放速度控制
        speed_layout = QHBoxLayout()
        speed_label = QLabel("播放速度:")
        self.speed_combo = QComboBox()
        self.speed_combo.addItems(["0.5x", "1.0x", "2.0x", "5.0x"])
        self.speed_combo.setCurrentText("1.0x")
        self.speed_combo.setMinimumWidth(80)

        speed_layout.addWidget(speed_label)
        speed_layout.addWidget(self.speed_combo)
        speed_layout.addStretch()  # 添加弹性空间
        control_layout.addLayout(speed_layout)

        control_group.setLayout(control_layout)
        layout.addWidget(control_group)

        # 添加状态显示
        self.status_label = QLabel("就绪")
        self.status_label.setAlignment(Qt.AlignLeft)
        layout.addWidget(self.status_label)

        # 添加弹性空间
        layout.addStretch()

        # 定时器设置
        self.timer = QTimer()
        self.timer.timeout.connect(self.update_playback)

    def browse_file(self):
        """选择回放文件"""
        start_dir = self.data_dir if self.data_dir else ""
        filename, _ = QFileDialog.getOpenFileName(
            self,
            "选择数据文件",
            start_dir,
            "CSV Files (*.csv)"
        )
        if filename:
            self.file_path.setText(filename)
            self.load_data(filename)

    def load_data(self, filename):
        """加载数据文件"""
        try:
            self.data = pd.read_csv(filename)
            self.slider.setRange(0, len(self.data)-1)
            self.current_index = 0
            self.slider.setValue(0)
            status_text = f"已加载 {len(self.data)} 条数据记录"
            self.status_label.setText(status_text)
            QMessageBox.information(self, "加载成功", status_text)
        except Exception as e:
            error_text = f"加载数据失败: {str(e)}"
            self.status_label.setText(error_text)
            QMessageBox.critical(self, "错误", error_text)

    def toggle_playback(self, playing):
        """切换播放状态"""
        self.playing = playing
        if playing:
            self.play_btn.setText("暂停")
            speed = float(self.speed_combo.currentText().replace('x', ''))
            self.timer.start(int(100/speed))  # 基础更新率10Hz
        else:
            self.play_btn.setText("播放")
            self.timer.stop()

    def update_playback(self):
        """更新回放数据"""
        if self.data is not None and self.current_index < len(self.data):
            self.data_updated.emit(self.data.iloc[self.current_index])
            self.current_index += 1
            self.slider.setValue(self.current_index)
            # 更新状态
            self.status_label.setText(f"正在播放: {self.current_index}/{len(self.data)}")
        else:
            self.play_btn.setChecked(False)
            self.status_label.setText("播放完成")

    def seek(self, value):
        """跳转到指定位置"""
        self.current_index = value
        if self.data is not None:
            self.data_updated.emit(self.data.iloc[value])
