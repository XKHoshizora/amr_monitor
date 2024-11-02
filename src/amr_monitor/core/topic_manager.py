# src/amr_monitor/core/topic_manager.py
"""ROS话题管理器"""
import threading
from typing import Dict, Any, Optional, Callable
import rospy
from amr_monitor.core.logger import Logger
from amr_monitor.core.event_bus import EventBus, Event
from amr_monitor.core.error_handler import handle_error, ErrorType, ErrorSeverity

logger = Logger.get_logger(__name__)

class TopicSubscription:
    """话题订阅信息"""
    def __init__(self,
                 topic_name: str,
                 msg_type: Any,
                 callback: Callable,
                 monitor_name: str,
                 queue_size: int = 10):
        self.topic_name = topic_name
        self.msg_type = msg_type
        self.callback = callback
        self.monitor_name = monitor_name
        self.queue_size = queue_size
        self.subscriber = None
        self.is_active = False
        self.error_count = 0
        self.max_retries = 3

class TopicManager:
    """话题管理器"""
    _instance = None
    _lock = threading.Lock()

    def __new__(cls):
        if cls._instance is None:
            with cls._lock:
                if cls._instance is None:
                    cls._instance = super(TopicManager, cls).__new__(cls)
        return cls._instance

    def __init__(self):
        if not hasattr(self, '_initialized'):
            self._initialized = True
            self.subscriptions: Dict[str, TopicSubscription] = {}
            self.event_bus = EventBus()
            self._lock = threading.Lock()

    def subscribe(self, subscription: TopicSubscription) -> bool:
        """订阅话题"""
        with self._lock:
            try:
                # 如果已存在订阅，先取消
                self.unsubscribe(subscription.topic_name)

                # 创建回调包装器
                def callback_wrapper(msg):
                    try:
                        if subscription.is_active:
                            subscription.callback(msg)
                            subscription.error_count = 0  # 重置错误计数
                    except Exception as e:
                        subscription.error_count += 1
                        if subscription.error_count >= subscription.max_retries:
                            self._handle_subscription_failure(subscription)
                        error_msg = f"Callback error for {subscription.topic_name}: {str(e)}"
                        handle_error(ErrorType.TOPIC, ErrorSeverity.ERROR, error_msg,
                                   subscription.monitor_name)

                # 创建订阅者
                subscription.subscriber = rospy.Subscriber(
                    subscription.topic_name,
                    subscription.msg_type,
                    callback_wrapper,
                    queue_size=subscription.queue_size
                )
                subscription.is_active = True
                self.subscriptions[subscription.topic_name] = subscription

                logger.info(f"Subscribed to topic: {subscription.topic_name}")
                return True

            except Exception as e:
                error_msg = f"Failed to subscribe to {subscription.topic_name}: {str(e)}"
                handle_error(ErrorType.TOPIC, ErrorSeverity.ERROR, error_msg,
                           subscription.monitor_name)
                return False

    def unsubscribe(self, topic_name: str):
        """取消订阅"""
        with self._lock:
            if topic_name in self.subscriptions:
                try:
                    sub = self.subscriptions[topic_name]
                    if sub.subscriber:
                        sub.subscriber.unregister()
                    sub.is_active = False
                    del self.subscriptions[topic_name]
                    logger.info(f"Unsubscribed from topic: {topic_name}")
                except Exception as e:
                    error_msg = f"Error unsubscribing from {topic_name}: {str(e)}"
                    handle_error(ErrorType.TOPIC, ErrorSeverity.WARNING, error_msg,
                               self.subscriptions[topic_name].monitor_name)

    def _handle_subscription_failure(self, subscription: TopicSubscription):
        """处理订阅失败"""
        error_msg = f"Subscription failed for {subscription.topic_name} after {subscription.error_count} errors"
        handle_error(ErrorType.TOPIC, ErrorSeverity.ERROR, error_msg,
                    subscription.monitor_name)

        # 尝试重新订阅
        self.unsubscribe(subscription.topic_name)
        if self.subscribe(subscription):
            logger.info(f"Successfully resubscribed to {subscription.topic_name}")
        else:
            logger.error(f"Failed to resubscribe to {subscription.topic_name}")

    def get_topic_info(self, topic_name: str) -> Dict[str, Any]:
        """获取话题信息"""
        if topic_name in self.subscriptions:
            sub = self.subscriptions[topic_name]
            return {
                'is_active': sub.is_active,
                'error_count': sub.error_count,
                'monitor_name': sub.monitor_name
            }
        return {}

    def list_active_topics(self) -> list:
        """列出所有活动话题"""
        return [topic for topic, sub in self.subscriptions.items() if sub.is_active]

    def cleanup(self):
        """清理所有订阅"""
        with self._lock:
            topics = list(self.subscriptions.keys())
            for topic in topics:
                self.unsubscribe(topic)

    def get_available_topics(self, msg_type: str = None) -> list:
        """获取可用话题列表"""
        topics = []
        for topic, topic_type in rospy.get_published_topics():
            if msg_type is None or topic_type == msg_type:
                topics.append(topic)
        return topics

# 创建全局实例
topic_manager = TopicManager()