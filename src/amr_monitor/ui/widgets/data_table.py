# src/amr_monitor/ui/widgets/data_table.py
"""数据表格控件实现"""
from typing import List, Dict, Any, Optional
from PySide6.QtWidgets import (QTableWidget, QTableWidgetItem, QHeaderView,
                               QVBoxLayout, QMenu)
from PySide6.QtCore import Qt, Signal
from PySide6.QtGui import QAction
import csv
from pathlib import Path

from .base_widget import BaseWidget


class DataTableWidget(BaseWidget):
    """数据表格控件"""

    # 自定义信号
    data_selected = Signal(dict)  # 选中数据时发送
    data_changed = Signal()      # 数据改变时发送

    def __init__(self, headers: List[str], parent=None):
        super().__init__(parent)
        self.headers = headers
        self.data: List[Dict[str, Any]] = []
        self.setup_ui()
        self.setup_context_menu()

    def setup_ui(self):
        """设置UI"""
        layout = QVBoxLayout(self)
        layout.setContentsMargins(0, 0, 0, 0)

        # 创建表格
        self.table = QTableWidget()
        self.table.setColumnCount(len(self.headers))
        self.table.setHorizontalHeaderLabels(self.headers)

        # 设置表格属性
        self.table.setAlternatingRowColors(True)
        self.table.setSelectionBehavior(QTableWidget.SelectRows)
        self.table.setEditTriggers(QTableWidget.NoEditTriggers)

        # 设置表头
        header = self.table.horizontalHeader()
        header.setSectionResizeMode(QHeaderView.ResizeToContents)
        header.setStretchLastSection(True)

        # 信号连接
        self.table.itemSelectionChanged.connect(self._on_selection_changed)

        layout.addWidget(self.table)

    def setup_context_menu(self):
        """设置右键菜单"""
        self.table.setContextMenuPolicy(Qt.CustomContextMenu)
        self.table.customContextMenuRequested.connect(self._show_context_menu)

        self.context_menu = QMenu(self)
        self.context_menu.addAction("Copy", self._copy_selection)
        self.context_menu.addAction("Export", self._export_data)
        self.context_menu.addSeparator()
        self.context_menu.addAction("Clear", self.clear)

    def update_data(self, data: List[Dict[str, Any]], clear: bool = True):
        """更新数据"""
        if clear:
            self.data.clear()
        self.data.extend(data)

        self._update_table()
        self.data_changed.emit()

    def _update_table(self):
        """更新表格显示"""
        self.table.setRowCount(len(self.data))

        for row, row_data in enumerate(self.data):
            for col, header in enumerate(self.headers):
                if header in row_data:
                    value = row_data[header]
                    # 格式化数值
                    if isinstance(value, float):
                        text = f"{value:.3f}"
                    else:
                        text = str(value)

                    item = QTableWidgetItem(text)
                    item.setTextAlignment(Qt.AlignCenter)
                    self.table.setItem(row, col, item)

    def _on_selection_changed(self):
        """处理选择变更"""
        rows = set(item.row() for item in self.table.selectedItems())
        if len(rows) == 1:
            row = rows.pop()
            if 0 <= row < len(self.data):
                self.data_selected.emit(self.data[row])

    def _show_context_menu(self, pos):
        """显示右键菜单"""
        self.context_menu.exec_(self.table.viewport().mapToGlobal(pos))

    def _copy_selection(self):
        """复制选中内容"""
        selection = self.table.selectedRanges()
        if not selection:
            return

        text = []
        for r in range(selection[0].topRow(), selection[0].bottomRow() + 1):
            row = []
            for c in range(selection[0].leftColumn(), selection[0].rightColumn() + 1):
                item = self.table.item(r, c)
                row.append(item.text() if item else '')
            text.append('\t'.join(row))

        QApplication.clipboard().setText('\n'.join(text))

    def _export_data(self):
        """导出数据"""
        if not self.data:
            return

        try:
            from PySide6.QtWidgets import QFileDialog
            filename, _ = QFileDialog.getSaveFileName(
                self,
                "Export Data",
                "",
                "CSV Files (*.csv);;All Files (*)"
            )

            if filename:
                with open(filename, 'w', newline='') as f:
                    writer = csv.DictWriter(f, fieldnames=self.headers)
                    writer.writeheader()
                    writer.writerows(self.data)

        except Exception as e:
            from ...core.error_handler import handle_error, ErrorType, ErrorSeverity
            handle_error(
                ErrorType.UI,
                ErrorSeverity.ERROR,
                f"Failed to export data: {str(e)}",
                "DataTableWidget"
            )

    def clear(self):
        """清空表格"""
        self.data.clear()
        self.table.setRowCount(0)
        self.data_changed.emit()

    def get_data(self) -> List[Dict[str, Any]]:
        """获取数据"""
        return self.data.copy()

    def set_column_width(self, column: int, width: int):
        """设置列宽"""
        self.table.setColumnWidth(column, width)

    def set_sorting_enabled(self, enabled: bool):
        """设置排序功能"""
        self.table.setSortingEnabled(enabled)

    def get_selected_rows(self) -> List[int]:
        """获取选中的行"""
        return sorted(set(item.row() for item in self.table.selectedItems()))

    def get_selected_data(self) -> List[Dict[str, Any]]:
        """获取选中的数据"""
        return [self.data[row] for row in self.get_selected_rows()]

    def set_row_colors(self, colors: Dict[int, str]):
        """设置行颜色"""
        for row, color in colors.items():
            if 0 <= row < self.table.rowCount():
                for col in range(self.table.columnCount()):
                    item = self.table.item(row, col)
                    if item:
                        item.setBackground(color)

    def set_column_alignment(self, column: int, alignment: Qt.Alignment):
        """设置列对齐方式"""
        if 0 <= column < self.table.columnCount():
            for row in range(self.table.rowCount()):
                item = self.table.item(row, column)
                if item:
                    item.setTextAlignment(alignment)

    def resize_columns_to_contents(self):
        """自适应列宽"""
        self.table.resizeColumnsToContents()

    def set_header_style(self, style_sheet: str):
        """设置表头样式"""
        self.table.horizontalHeader().setStyleSheet(style_sheet)

    def set_alternating_row_colors(self, enabled: bool):
        """设置交替行颜色"""
        self.table.setAlternatingRowColors(enabled)

    def get_column_index(self, header: str) -> int:
        """获取列索引"""
        return self.headers.index(header) if header in self.headers else -1

    def set_value(self, row: int, column: int, value: Any):
        """设置单元格值"""
        if 0 <= row < self.table.rowCount() and 0 <= column < self.table.columnCount():
            # 更新数据
            header = self.headers[column]
            if 0 <= row < len(self.data):
                self.data[row][header] = value

            # 更新显示
            if isinstance(value, float):
                text = f"{value:.3f}"
            else:
                text = str(value)

            item = QTableWidgetItem(text)
            item.setTextAlignment(Qt.AlignCenter)
            self.table.setItem(row, column, item)

            self.data_changed.emit()

    def add_row(self, row_data: Dict[str, Any]):
        """添加行"""
        self.data.append(row_data)
        self._update_table()
        self.data_changed.emit()

    def remove_rows(self, rows: List[int]):
        """删除行"""
        for row in sorted(rows, reverse=True):
            if 0 <= row < len(self.data):
                self.data.pop(row)
        self._update_table()
        self.data_changed.emit()

    def set_max_rows(self, max_rows: int):
        """设置最大行数"""
        while len(self.data) > max_rows:
            self.data.pop(0)
        self._update_table()
