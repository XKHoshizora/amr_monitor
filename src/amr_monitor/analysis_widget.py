from PyQt5.QtWidgets import *
from PyQt5.QtCore import *
import numpy as np
import pandas as pd
import pyqtgraph as pg
from scipy import signal
from scipy.stats import pearsonr


class AnalysisWidget(QWidget):
    def __init__(self, data_dir=None, parent=None):
        super().__init__(parent, Qt.Window)  # 使用Qt.Window标志
        self.setAttribute(Qt.WA_DeleteOnClose, False)  # 不自动删除
        self.setWindowTitle("数据分析")
        self.setMinimumSize(800, 600)
        self.data_dir = data_dir
        self.setup_ui()

    def setup_ui(self):
        layout = QVBoxLayout(self)

        # 文件选择组
        file_group = QGroupBox("数据文件")
        file_layout = QHBoxLayout()
        self.file_path = QLineEdit()
        browse_btn = QPushButton("浏览")
        browse_btn.clicked.connect(self.browse_file)
        file_layout.addWidget(self.file_path)
        file_layout.addWidget(browse_btn)
        file_group.setLayout(file_layout)
        layout.addWidget(file_group)

        # 数据选择
        data_group = QGroupBox("数据选择")
        data_layout = QHBoxLayout()

        self.data_combo = QComboBox()
        self.data_combo.addItems([
            "IMU角速度",
            "IMU线加速度",
            "里程计位置",
            "里程计速度"
        ])

        data_layout.addWidget(QLabel("数据类型:"))
        data_layout.addWidget(self.data_combo)
        data_group.setLayout(data_layout)
        layout.addWidget(data_group)

        # 分析工具选择
        tool_group = QGroupBox("分析工具")
        tool_layout = QHBoxLayout()

        self.tool_combo = QComboBox()
        self.tool_combo.addItems([
            "基础统计",
            "频谱分析",
            "相关性分析",
            "异常检测"
        ])

        analyze_btn = QPushButton("分析")
        analyze_btn.clicked.connect(self.perform_analysis)

        tool_layout.addWidget(QLabel("分析方法:"))
        tool_layout.addWidget(self.tool_combo)
        tool_layout.addWidget(analyze_btn)
        tool_group.setLayout(tool_layout)
        layout.addWidget(tool_group)

        # 结果显示区域
        results_group = QGroupBox("分析结果")
        results_layout = QVBoxLayout()

        self.result_text = QTextEdit()
        self.result_text.setReadOnly(True)

        self.plot_widget = pg.PlotWidget()
        self.plot_widget.setBackground('w')

        results_layout.addWidget(self.result_text)
        results_layout.addWidget(self.plot_widget)
        results_group.setLayout(results_layout)
        layout.addWidget(results_group)

    def browse_file(self):
        """浏览数据文件"""
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

    def perform_analysis(self):
        """执行数据分析"""
        data_type = self.data_combo.currentText()
        analysis_type = self.tool_combo.currentText()

        # 获取数据
        data = self.get_analysis_data(data_type)
        if data is None:
            return

        # 执行分析
        if analysis_type == "基础统计":
            self.basic_statistics(data)
        elif analysis_type == "频谱分析":
            self.spectral_analysis(data)
        elif analysis_type == "相关性分析":
            self.correlation_analysis(data)
        elif analysis_type == "异常检测":
            self.anomaly_detection(data)

    def get_analysis_data(self, data_type):
        """获取指定类型的数据"""
        try:
            if data_type == "IMU角速度":
                return self.parent().plot_widget.imu_data['angular_velocity']
            elif data_type == "IMU线加速度":
                return self.parent().plot_widget.imu_data['linear_acceleration']
            elif data_type == "里程计位置":
                return self.parent().plot_widget.odom_data['position']
            elif data_type == "里程计速度":
                return self.parent().plot_widget.odom_data['velocity']
        except AttributeError:
            QMessageBox.warning(self, "警告", "无法获取数据")
            return None

    def basic_statistics(self, data):
        """基础统计分析"""
        stats = {}
        for axis, values in data.items():
            stats[axis] = {
                '均值': np.mean(values),
                '标准差': np.std(values),
                '最小值': np.min(values),
                '最大值': np.max(values),
                '中位数': np.median(values)
            }

        # 显示结果
        result_text = "基础统计分析结果:\\n\\n"
        for axis, stat in stats.items():
            result_text += f"{axis}轴:\\n"
            for name, value in stat.items():
                result_text += f"  {name}: {value:.4f}\\n"
            result_text += "\\n"

        self.result_text.setText(self.format_results(result_text))

        # 绘制直方图
        self.plot_widget.clear()
        colors = {'x': 'r', 'y': 'g', 'z': 'b', 'linear': 'c', 'angular': 'm'}
        for axis, values in data.items():
            if axis in colors:
                y, x = np.histogram(values, bins=50)
                self.plot_widget.plot(x, y, stepMode=True, fillLevel=0,
                                      pen=colors[axis], name=axis)

    def spectral_analysis(self, data):
        """频谱分析"""
        self.plot_widget.clear()

        # 定义颜色映射
        colors = {'x': (255, 0, 0), 'y': (0, 255, 0), 'z': (0, 0, 255),
                'linear': (0, 255, 255), 'angular': (255, 0, 255)}

        # 对每个轴进行FFT分析
        for axis, values in data.items():
            # 计算采样率
            dt = 0.1  # 假设10Hz采样率
            n = len(values)
            freq = np.fft.fftfreq(n, dt)

            # 计算FFT
            fft = np.fft.fft(values)
            magnitude = np.abs(fft)

            # 只显示正频率部分
            pos_freq = freq[1:n//2]
            pos_magnitude = magnitude[1:n//2]

            # 使用预定义的颜色
            color = colors.get(axis, (128, 128, 128))  # 如果没有定义颜色，使用灰色

            # 绘制频谱
            self.plot_widget.plot(pos_freq, pos_magnitude,
                            pen=color,
                            name=f"{axis}轴")

    def correlation_analysis(self, data):
        """相关性分析"""
        # 计算各轴之间的相关系数
        correlations = {}
        axes = list(data.keys())
        for i in range(len(axes)):
            for j in range(i+1, len(axes)):
                axis1, axis2 = axes[i], axes[j]
                corr, p_value = pearsonr(data[axis1], data[axis2])
                correlations[f"{axis1}-{axis2}"] = (corr, p_value)

        # 显示结果
        result_text = "相关性分析结果:\\n\\n"
        for pair, (corr, p_value) in correlations.items():
            result_text += f"{pair}:\\n"
            result_text += f"  相关系数: {corr:.4f}\\n"
            result_text += f"  P值: {p_value:.4f}\\n\\n"

        self.result_text.setText(self.format_results(result_text))

        # 绘制散点图
        self.plot_widget.clear()
        for pair in correlations.keys():
            axis1, axis2 = pair.split('-')
            scatter = pg.ScatterPlotItem(
                x=data[axis1],
                y=data[axis2],
                pen=None,
                symbol='o',
                size=5,
                brush=pg.mkBrush(0, 0, 255, 128)
            )
            self.plot_widget.addItem(scatter)
            self.plot_widget.setLabel('left', axis2)
            self.plot_widget.setLabel('bottom', axis1)
            break  # 只显示第一组数据的散点图

    def anomaly_detection(self, data):
        """异常检测"""
        anomalies = {}
        for axis, values in data.items():
            # 使用3倍标准差作为异常判断标准
            mean = np.mean(values)
            std = np.std(values)
            threshold = 3 * std

            # 检测异常值
            anomaly_indices = np.where(np.abs(values - mean) > threshold)[0]
            anomalies[axis] = {
                'indices': anomaly_indices,
                'values': values[anomaly_indices]
            }

        # 显示结果
        result_text = "异常检测结果:\\n\\n"
        for axis, result in anomalies.items():
            result_text += f"{axis}轴:\\n"
            result_text += f"  检测到 {len(result['indices'])} 个异常点\\n"
            if len(result['indices']) > 0:
                result_text += "  异常值:\\n"
                for idx, val in zip(result['indices'][:5], result['values'][:5]):
                    result_text += f"    位置 {idx}: {val:.4f}\\n"
                if len(result['indices']) > 5:
                    result_text += "    ...\\n"
            result_text += "\\n"

        self.result_text.setText(self.format_results(result_text))

        # 绘制异常检测图
        self.plot_widget.clear()
        colors = {'x': 'r', 'y': 'g', 'z': 'b', 'linear': 'c', 'angular': 'm'}
        for axis, result in anomalies.items():
            if axis in colors:
                # 绘制原始数据
                self.plot_widget.plot(data[axis], pen=colors[axis], name=axis)
                # 标记异常点
                scatter = pg.ScatterPlotItem(
                    x=result['indices'],
                    y=result['values'],
                    pen=None,
                    symbol='o',
                    size=8,
                    brush=pg.mkBrush(255, 0, 0, 200)
                )
                self.plot_widget.addItem(scatter)

    def format_results(self, text):
        """格式化结果文本"""
        # 确保使用系统换行符
        text = text.replace('\n', os.linesep)
        # 添加适当的缩进
        lines = text.split(os.linesep)
        formatted_lines = []
        indent = 0
        for line in lines:
            if line.strip().endswith(':'):
                formatted_lines.append('  ' * indent + line)
                indent += 1
            else:
                formatted_lines.append('  ' * indent + line)
                if not line.strip():
                    indent = max(0, indent - 1)
        return os.linesep.join(formatted_lines)
