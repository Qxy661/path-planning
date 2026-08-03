"""
TurtleBot3 自动建图脚本（SLAM）

发布预定义轨迹让小车遍历室内，构建地图。
轨迹：直线前进 → 转弯 → 前进 → 转弯 ...（房间巡检路径）

Usage:
    python3 auto_explore.py
"""
import math
import time

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist


class AutoExplore(Node):
    def __init__(self):
        super().__init__("auto_explore")
        self.pub = self.create_publisher(Twist, "cmd_vel", 10)
        self.get_logger().info("自动建图启动: 发布移动轨迹")

    def move(self, linear, angular, duration):
        """发布速度指令持续 duration 秒."""
        twist = Twist()
        twist.linear.x = linear
        twist.angular.z = angular
        self.pub.publish(twist)
        time.sleep(duration)

    def explore(self):
        """巡检轨迹：矩形路线遍历室内."""
        # 前进 5 秒
        self.move(0.15, 0.0, 5.0)
        # 右转 3 秒
        self.move(0.0, 0.5, 3.0)
        # 前进 4 秒
        self.move(0.15, 0.0, 4.0)
        # 右转
        self.move(0.0, 0.5, 3.0)
        # 前进 5 秒
        self.move(0.15, 0.0, 5.0)
        # 右转
        self.move(0.0, 0.5, 3.0)
        # 前进 4 秒
        self.move(0.15, 0.0, 4.0)
        # 右转 (回到起点附近)
        self.move(0.0, 0.5, 3.0)
        # 停止
        self.move(0.0, 0.0, 0.5)
        self.get_logger().info("建图轨迹完成")
        self.get_logger().info("请保存地图: ros2 run nav2_map_server map_saver_cli -f ~/map")


def main():
    rclpy.init()
    explorer = AutoExplore()
    try:
        explorer.explore()
    except KeyboardInterrupt:
        pass
    finally:
        explorer.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
