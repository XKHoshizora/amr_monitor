"""事件总线模块"""
from dataclasses import dataclass, field
from typing import Dict, List, Any, Callable
import threading
import time
import rospy

@dataclass
class Event:
    """事件类"""
    type: str
    data: Any
    source: str
    timestamp: float = field(default_factory=time.time)

class EventBus:
    """事件总线"""

    def __init__(self):
        self._subscribers: Dict[str, List[Callable]] = {}
        self._lock = threading.Lock()

    def subscribe(self, event_type: str, callback: Callable[[Event], None]):
        """订阅事件"""
        with self._lock:
            if event_type not in self._subscribers:
                self._subscribers[event_type] = []
            self._subscribers[event_type].append(callback)

    def unsubscribe(self, event_type: str, callback: Callable):
        """取消订阅"""
        with self._lock:
            if event_type in self._subscribers:
                try:
                    self._subscribers[event_type].remove(callback)
                except ValueError:
                    pass

    def publish(self, event: Event):
        """发布事件"""
        with self._lock:
            if event.type in self._subscribers:
                for callback in self._subscribers[event.type]:
                    try:
                        callback(event)
                    except Exception as e:
                        rospy.logerr(f"Error in event callback: {e}")

    def clear(self):
        """清空所有订阅"""
        with self._lock:
            self._subscribers.clear()