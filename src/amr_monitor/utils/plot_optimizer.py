import pyqtgraph as pg
import numpy as np


class PlotOptimizer:
    """图表优化器"""
    @staticmethod
    def optimize_plot(plot_widget):
        """优化图表性能"""
        # 启用OpenGL加速
        plot_widget.useOpenGL(True)

        # 设置抗锯齿
        plot_widget.setAntialiasing(True)

        # 启用下采样
        plot_widget.setDownsampling(auto=True, mode='peak')

        # 启用有限内存模式
        plot_widget.setClipToView(True)

    @staticmethod
    def create_optimized_curve(plot_widget, **kwargs):
        """创建优化的曲线"""
        curve = plot_widget.plot(**kwargs)

        # 设置曲线特性
        curve.setDownsampling(auto=True, mode='peak')
        curve.setClipToView(True)

        return curve
