"""统一优化管理器"""
from dataclasses import dataclass
from typing import Dict, Any, Optional
from abc import ABC, abstractmethod
import threading
import time
import rospy

@dataclass
class OptimizationConfig:
    """优化配置"""
    enabled: bool = True
    update_interval: float = 1.0
    thresholds: Dict[str, float] = None

class OptimizationStrategy(ABC):
    """优化策略基类"""

    def __init__(self, config: OptimizationConfig):
        self.config = config
        self.enabled = config.enabled

    def optimize(self, target: Any) -> bool:
        """执行优化"""
        if not self.enabled:
            return False
        return self._do_optimize(target)

    @abstractmethod
    def _do_optimize(self, target: Any) -> bool:
        """执行具体优化"""
        pass

class PerformanceStrategy(OptimizationStrategy):
    """性能优化策略"""

    def _do_optimize(self, target: Any) -> bool:
        try:
            # CPU优化
            if hasattr(target, 'process'):
                target.process.nice(10)

            # 内存优化
            if hasattr(target, 'clear_cache'):
                target.clear_cache()

            return True
        except Exception as e:
            rospy.logerr(f"Performance optimization failed: {e}")
            return False

class UIStrategy(OptimizationStrategy):
    """UI优化策略"""

    def _do_optimize(self, target: Any) -> bool:
        try:
            # 减少更新频率
            if hasattr(target, 'update_interval'):
                target.update_interval *= 1.5

            # 启用双缓冲
            if hasattr(target, 'enable_double_buffering'):
                target.enable_double_buffering()

            # 优化渲染
            if hasattr(target, 'optimize_rendering'):
                target.optimize_rendering()

            return True
        except Exception as e:
            rospy.logerr(f"UI optimization failed: {e}")
            return False

class StorageStrategy(OptimizationStrategy):
    """存储优化策略"""

    def _do_optimize(self, target: Any) -> bool:
        try:
            # 压缩数据
            if hasattr(target, 'compress_data'):
                target.compress_data()

            # 清理旧数据
            if hasattr(target, 'cleanup_old_data'):
                target.cleanup_old_data()

            return True
        except Exception as e:
            rospy.logerr(f"Storage optimization failed: {e}")
            return False

class OptimizationManager:
    """优化管理器"""

    def __init__(self):
        self.strategies = {}
        self._lock = threading.Lock()
        self._init_strategies()

    def _init_strategies(self):
        """初始化优化策略"""
        # 性能优化
        perf_config = OptimizationConfig(
            thresholds={
                'cpu': 80.0,
                'memory': 80.0
            }
        )
        self.add_strategy('performance', PerformanceStrategy(perf_config))

        # UI优化
        ui_config = OptimizationConfig(
            update_interval=0.05
        )
        self.add_strategy('ui', UIStrategy(ui_config))

        # 存储优化
        storage_config = OptimizationConfig(
            update_interval=60.0
        )
        self.add_strategy('storage', StorageStrategy(storage_config))

    def add_strategy(self, name: str, strategy: OptimizationStrategy):
        """添加优化策略"""
        with self._lock:
            self.strategies[name] = strategy

    def optimize(self, target: Any, strategy_name: str) -> bool:
        """执行优化"""
        with self._lock:
            if strategy_name not in self.strategies:
                return False
            return self.strategies[strategy_name].optimize(target)

    def optimize_all(self, target: Any) -> Dict[str, bool]:
        """执行所有优化"""
        results = {}
        for name, strategy in self.strategies.items():
            results[name] = self.optimize(target, name)
        return results