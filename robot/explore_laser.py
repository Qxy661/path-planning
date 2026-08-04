#!/usr/bin/env python3
"""
基于激光的自动建图探索（墙壁跟随 / 避障）

读取 /scan 判断前方是否被阻挡，若无障碍则前进，
遇障则转向，天然遍历迷宫通道。用于 Cartographer 建图。

Usage:
    python3 explore_laser.py            # 默认跑 90 秒
    python3 explore_laser.py 120        # 指定时长
"""
import math
import sys
import time

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy


class LaserExplorer(Node):
    def __init__(self, duration=90):
        super().__init__("explore_laser")
        # gazebo 激光为 BEST_EFFORT，订阅需匹配
        scan_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=5)
        self.pub = self.create_publisher(Twist, "cmd_vel", 10)
        self.sub = self.create_subscription(LaserScan, "scan", self.scan_cb, scan_qos)
        self.laser = None
        self.duration = duration
        self.get_logger().info(f"激光自主探索启动，预计 {duration}s")

    def scan_cb(self, msg):
        self.laser = msg

    def front_dist(self):
        """前方最近障碍距离（±30° 扇区取最小）."""
        if self.laser is None or len(self.laser.ranges) == 0:
            return None
        n = len(self.laser.ranges)
        # 前方扇区：-30° ~ +30°
        ranges = self.laser.ranges
        mid = n // 2
        sector = ranges[mid - n // 6: mid + n // 6 + 1]
        valid = [r for r in sector if 0 < r < self.laser.range_max]
        return min(valid) if valid else self.laser.range_max

    def explore(self):
        start = time.time()
        state = "forward"
        turn_start = None
        while rclpy.ok() and time.time() - start < self.duration:
            d = self.front_dist()
            twist = Twist()

            if d is None:
                # 还没有 scan 数据，原地等待
                twist.linear.x = 0.0
                self.pub.publish(twist)
                rclpy.spin_once(self, timeout_sec=0.2)
                continue

            if d < 0.45:
                # 前方受阻 → 原地右转，直到前方通畅
                state = "turning"
                twist.angular.z = 0.5
                twist.linear.x = 0.0
            elif state == "turning":
                # 刚脱困，短暂回正后前进
                state = "forward"
                twist.linear.x = 0.15
            else:
                # 通畅 → 前进
                twist.linear.x = 0.15
                # 轻微修正：右前方更近则略微左偏（避免贴着墙蹭）
                twist.angular.z = 0.0

            self.pub.publish(twist)
            rclpy.spin_once(self, timeout_sec=0.05)
            time.sleep(0.05)

        # 停止
        self.pub.publish(Twist())
        self.get_logger().info("探索完成")


def main():
    rclpy.init()
    duration = int(sys.argv[1]) if len(sys.argv) > 1 else 90
    explorer = LaserExplorer(duration)
    try:
        explorer.explore()
    except KeyboardInterrupt:
        pass
    finally:
        explorer.pub.publish(Twist())
        explorer.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
