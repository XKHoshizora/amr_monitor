# src/amr_monitor/ui/widgets/plot_widget.py
"""绘图控件实现"""
from typing import Dict, Optional, List, Tuple
import numpy as np
import pyqtgraph as pg
from PySide6.QtWidgets import QWidget, QVBoxLayout, QLabel
from PySide6.QtCore import Qt

from .base_widget import BaseWidget


class PlotWidget(BaseWidget):
    """绘图控件"""

    def __init__(self, title: str = "", parent: Optional[QWidget] = None):
        super().__init__(parent)
        self.title = title
        self.plot: Optional[pg.PlotWidget] = None
        self.curves: Dict[str, pg.PlotDataItem] = {}
        self.data_buffers: Dict[str, Tuple[List, List]] = {}  # (x, y) 数据缓冲
        self.max_points = 1000  # 最大数据点数

        self.setup_ui()
        self.setup_plot()

    def setup_ui(self):
        """设置UI"""
        layout = QVBoxLayout(self)
        layout.setContentsMargins(0, 0, 0, 0)

        # 标题标签
        if self.title:
            title_label = QLabel(self.title)
            title_label.setAlignment(Qt.AlignCenter)
            layout.addWidget(title_label)

        # 图表占位
        self.plot_container = QWidget()
        layout.addWidget(self.plot_container)

    def setup_plot(self):
        """设置图表"""
        self.plot = pg.PlotWidget()

        # 基本设置
        self.plot.showGrid(x=True, y=True, alpha=0.3)
        self.plot.setBackground('default')
        self.plot.setAntialiasing(True)

        # 添加到布局
        layout = QVBoxLayout(self.plot_container)
        layout.setContentsMargins(0, 0, 0, 0)
        layout.addWidget(self.plot)

    def add_curve(self, name: str, color: Optional[str] = None,
                  width: int = 1) -> pg.PlotDataItem:
        """添加曲线"""
        if name in self.curves:
            return self.curves[name]

        pen = pg.mkPen(color=color, width=width) if color else None
        curve = self.plot.plot(name=name, pen=pen)
        self.curves[name] = curve
        self.data_buffers[name] = ([], [])  # 初始化数据缓冲

        return curve

    def update_curve(self, name: str, x_data: float, y_data: float):
        """更新曲线数据"""
        if name not in self.curves:
            self.add_curve(name)

        # 获取数据缓冲
        x_buffer, y_buffer = self.data_buffers[name]

        # 添加新数据
        x_buffer.append(x_data)
        y_buffer.append(y_data)

        # 限制数据点数
        if len(x_buffer) > self.max_points:
            x_buffer.pop(0)
            y_buffer.pop(0)

        # 更新曲线
        self.curves[name].setData(x_buffer, y_buffer)

    def update_curve_batch(self, name: str, x_data: List, y_data: List):
        """批量更新曲线数据"""
        if name not in self.curves:
            self.add_curve(name)

        if len(x_data) != len(y_data):
            return

        # 更新数据缓冲
        self.data_buffers[name] = (
            list(x_data[-self.max_points:]),
            list(y_data[-self.max_points:])
        )

        # 更新曲线
        self.curves[name].setData(x_data, y_data)

    def clear_curve(self, name: str):
        """清空曲线数据"""
        if name in self.curves:
            self.data_buffers[name] = ([], [])
            self.curves[name].clear()

    def clear_all(self):
        """清空所有曲线"""
        for name in self.curves:
            self.clear_curve(name)

    def set_title(self, title: str):
        """设置标题"""
        self.plot.setTitle(title)

    def set_labels(self, x_label: Optional[str] = None,
                   y_label: Optional[str] = None):
        """设置轴标签"""
        if x_label:
            self.plot.setLabel('bottom', x_label)
        if y_label:
            self.plot.setLabel('left', y_label)

    def set_range(self, x_range: Optional[Tuple[float, float]] = None,
                  y_range: Optional[Tuple[float, float]] = None):
        """设置显示范围"""
        if x_range:
            self.plot.setXRange(*x_range)
        if y_range:
            self.plot.setYRange(*y_range)

    def enable_auto_range(self, enable: bool = True):
        """启用/禁用自动范围"""
        self.plot.enableAutoRange(enable)

    def enable_legend(self, enable: bool = True):
        """启用/禁用图例"""
        if enable:
            self.plot.addLegend()
        else:
            if self.plot.legend:
                self.plot.legend.scene().removeItem(self.plot.legend)
