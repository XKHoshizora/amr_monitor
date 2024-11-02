"""配置管理模块"""
from dataclasses import dataclass, field
from typing import Dict, Any, Optional, List
from pathlib import Path
import yaml
import rospy

@dataclass
class QoSProfile:
    """QoS配置文件"""
    reliability: str = "reliable"
    durability: str = "volatile"
    depth: int = 10

@dataclass
class TopicConfig:
    """话题配置"""
    name: str
    msg_type: str
    qos: QoSProfile = field(default_factory=QoSProfile)
    filters: List[Dict] = field(default_factory=list)
    plotting: List[Dict] = field(default_factory=list)

@dataclass
class MonitorConfig:
    """监控配置"""
    update_interval: float = 0.1
    buffer_size: int = 1000
    save_data: bool = True
    data_dir: Optional[Path] = None
    alerts: Dict[str, float] = field(default_factory=dict)

@dataclass
class PerformanceConfig:
    """性能配置"""
    cpu_threshold: float = 80.0
    memory_threshold: float = 80.0
    update_interval: float = 1.0
    enable_optimization: bool = True

@dataclass
class UIConfig:
    """UI配置"""
    theme: str = "dark"
    layout: Dict[str, Any] = field(default_factory=dict)
    plots: Dict[str, Any] = field(default_factory=dict)

class ConfigManager:
    """配置管理器"""

    def __init__(self):
        self.config_path = Path(rospy.get_param('~config_path', 'config'))
        # 需要验证配置路径是否存在
        if not self.config_path.exists():
            raise FileNotFoundError(f"Config path not found: {self.config_path}")

        self.topic_configs: Dict[str, TopicConfig] = {}
        self.monitor_configs: Dict[str, MonitorConfig] = {}
        self.performance_config: PerformanceConfig = PerformanceConfig()
        self.ui_config: UIConfig = UIConfig()

        self.load_configs()

    def load_configs(self):
        """加载所有配置"""
        try:
            # 加载话题配置
            topics_dir = self.config_path / 'topics'
            for config_file in topics_dir.glob('*_config.yaml'):
                topic_type = config_file.stem.split('_')[0]
                config = self._load_yaml(config_file)
                self.topic_configs[topic_type] = TopicConfig(**config[topic_type])

            # 加载监控器配置
            monitor_config = self._load_yaml(self.config_path / 'monitor_config.yaml')
            for name, config in monitor_config.items():
                self.monitor_configs[name] = MonitorConfig(**config)

            # 加载性能配置
            perf_config = self._load_yaml(self.config_path / 'performance_config.yaml')
            self.performance_config = PerformanceConfig(**perf_config)

            # 加载UI配置
            ui_config = self._load_yaml(self.config_path / 'ui_config.yaml')
            self.ui_config = UIConfig(**ui_config)

        except Exception as e:
            rospy.logerr(f"Failed to load configs: {e}")

    def _load_yaml(self, file_path: Path) -> Dict:
        """加载YAML文件"""
        with open(file_path) as f:
            return yaml.safe_load(f)

    def get_topic_config(self, topic_type: str) -> Optional[TopicConfig]:
        """获取话题配置"""
        return self.topic_configs.get(topic_type)

    def get_monitor_config(self, name: str) -> Optional[MonitorConfig]:
        """获取监控器配置"""
        return self.monitor_configs.get(name)

    def save_config(self, name: str, config: Dict):
        """保存配置"""
        file_path = self.config_path / f'{name}_config.yaml'
        with open(file_path, 'w') as f:
            yaml.safe_dump(config, f)