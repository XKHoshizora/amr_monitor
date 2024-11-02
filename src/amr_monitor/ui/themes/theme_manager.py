# src/amr_monitor/ui/themes/theme_manager.py
"""主题管理实现"""
from pathlib import Path
from typing import Dict, Optional
from PySide6.QtWidgets import QWidget
from PySide6.QtCore import QFile
import rospy


class ThemeManager:
    """主题管理器"""

    def __init__(self):
        self.themes: Dict[str, str] = {}
        self.current_theme = None
        self.theme_dir = Path(rospy.get_param('~theme_path',
                                              str(Path(__file__).parent / 'styles')))
        self._load_themes()

    def _load_themes(self):
        """加载所有主题"""
        try:
            # 加载内置主题
            dark_theme = self._read_theme_file('dark_theme.qss')
            light_theme = self._read_theme_file('light_theme.qss')

            if dark_theme:
                self.themes['dark'] = dark_theme
            if light_theme:
                self.themes['light'] = light_theme

            # 加载用户自定义主题
            if self.theme_dir.exists():
                for theme_file in self.theme_dir.glob('*.qss'):
                    theme_name = theme_file.stem
                    theme_content = self._read_theme_file(theme_file)
                    if theme_content:
                        self.themes[theme_name] = theme_content

        except Exception as e:
            rospy.logerr(f"Failed to load themes: {e}")

    def _read_theme_file(self, file_name: str) -> Optional[str]:
        """读取主题文件"""
        try:
            # 首先检查内置主题路径
            builtin_path = Path(__file__).parent / 'styles' / file_name
            if builtin_path.exists():
                with open(builtin_path) as f:
                    return f.read()

            # 然后检查用户主题路径
            user_path = self.theme_dir / file_name
            if user_path.exists():
                with open(user_path) as f:
                    return f.read()

        except Exception as e:
            rospy.logerr(f"Failed to read theme file {file_name}: {e}")
        return None

    def apply_theme(self, widget: QWidget, name: str) -> bool:
        """应用主题到部件"""
        if name not in self.themes:
            rospy.logwarn(f"Theme {name} not found")
            return False

        try:
            widget.setStyleSheet(self.themes[name])
            self.current_theme = name
            return True
        except Exception as e:
            rospy.logerr(f"Failed to apply theme {name}: {e}")
            return False

    def get_current_theme(self) -> Optional[str]:
        """获取当前主题名称"""
        return self.current_theme

    def get_available_themes(self) -> list:
        """获取所有可用主题"""
        return list(self.themes.keys())

    def add_custom_theme(self, name: str, style_sheet: str) -> bool:
        """添加自定义主题"""
        try:
            # 验证样式表
            test_widget = QWidget()
            test_widget.setStyleSheet(style_sheet)

            # 保存主题
            self.themes[name] = style_sheet

            # 写入文件
            theme_path = self.theme_dir / f"{name}.qss"
            theme_path.write_text(style_sheet)

            return True
        except Exception as e:
            rospy.logerr(f"Failed to add custom theme {name}: {e}")
            return False
