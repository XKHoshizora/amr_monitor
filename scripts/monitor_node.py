#!/usr/bin/env python3

import sys
import signal
import rospy
from PyQt5.QtWidgets import QApplication, QMessageBox
from PyQt5.QtCore import QTimer
from amr_monitor.main_window import MainWindow


class MonitorNode:
    def __init__(self):
        # 初始化ROS节点
        rospy.init_node('amr_monitor', anonymous=True)

        # 获取参数
        self.update_rate = rospy.get_param('~update_rate', 10)  # 默认10Hz

        # 创建Qt应用
        self.app = QApplication(sys.argv)

        # 设置信号处理
        signal.signal(signal.SIGINT, self.signal_handler)

        # 创建主窗口
        try:
            self.main_window = MainWindow()
            self.main_window.show()
        except Exception as e:
            rospy.logerr(f"创建主窗口失败: {str(e)}")
            sys.exit(1)

        # 创建定时器处理ROS消息
        self.timer = QTimer()
        self.timer.timeout.connect(self.process_events)
        self.timer.start(int(1000/self.update_rate))  # 将频率转换为毫秒

    def signal_handler(self, signum, frame):
        """处理Ctrl+C信号"""
        rospy.loginfo("正在关闭监控节点...")
        self.app.quit()

    def process_events(self):
        """处理ROS消息和更新UI"""
        try:
            if not rospy.is_shutdown():
                rospy.spinOnce()
            else:
                self.shutdown()
        except Exception as e:
            rospy.logerr(f"处理消息时发生错误: {str(e)}")

    def shutdown(self):
        """关闭程序"""
        try:
            # 保存任何未保存的数据
            if hasattr(self.main_window, 'recording') and self.main_window.recording:
                self.main_window.toggle_recording(False)

            # 关闭窗口和应用
            self.main_window.close()
            self.app.quit()
        except Exception as e:
            rospy.logerr(f"关闭程序时发生错误: {str(e)}")

    def run(self):
        """运行监控节点"""
        try:
            rospy.loginfo("AMR监控节点已启动")
            return self.app.exec_()
        except Exception as e:
            rospy.logerr(f"运行程序时发生错误: {str(e)}")
            return 1


def main():
    try:
        monitor = MonitorNode()
        return monitor.run()
    except Exception as e:
        rospy.logerr(f"启动监控节点失败: {str(e)}")
        app = QApplication(sys.argv)
        QMessageBox.critical(None, "错误", f"启动监控节点失败: {str(e)}")
        return 1


if __name__ == '__main__':
    sys.exit(main())
