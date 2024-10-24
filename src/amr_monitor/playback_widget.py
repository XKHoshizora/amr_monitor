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
        super().__init__(parent)
        self.data_dir = data_dir
        self.setup_ui()
        self.data = None
        self.current_index = 0
        self.playing = False

    def setup_ui(self):
        layout = QVBoxLayout(self)

        # 文件选择
        file_group = QHBoxLayout()
        self.file_path = QLineEdit()
        browse_btn = QPushButton("浏览")
        browse_btn.clicked.connect(self.browse_file)
        file_group.addWidget(self.file_path)
        file_group.addWidget(browse_btn)
        layout.addLayout(file_group)

        # 播放控制
        control_group = QHBoxLayout()
        self.play_btn = QPushButton("播放")
        self.play_btn.setCheckable(True)
        self.play_btn.toggled.connect(self.toggle_playback)
        self.slider = QSlider(Qt.Horizontal)
        self.slider.valueChanged.connect(self.seek)
        control_group.addWidget(self.play_btn)
        control_group.addWidget(self.slider)
        layout.addLayout(control_group)

        # 播放速度控制
        speed_group = QHBoxLayout()
        speed_group.addWidget(QLabel("播放速度:"))
        self.speed_combo = QComboBox()
        self.speed_combo.addItems(["0.5x", "1.0x", "2.0x", "5.0x"])
        self.speed_combo.setCurrentText("1.0x")
        speed_group.addWidget(self.speed_combo)
        layout.addLayout(speed_group)

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
            QMessageBox.information(self, "加载成功",
                                    f"已加载 {len(self.data)} 条数据记录")
        except Exception as e:
            QMessageBox.critical(self, "错误", f"加载数据失败: {str(e)}")

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
        else:
            self.play_btn.setChecked(False)

    def seek(self, value):
        """跳转到指定位置"""
        self.current_index = value
        if self.data is not None:
            self.data_updated.emit(self.data.iloc[value])
