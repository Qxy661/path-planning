#!/usr/bin/env python3
"""
Nav2 导航量化指标采集：导航时记录全局规划路径长度、到达耗时。

与 exploration/astar_on_slam_map.py 的手写 A* 指标对照，
形成「手写算法 vs Nav2」的量化对比表。

用法:
    python3 nav_metrics.py                # 默认到 (1.5, 1.5)
    python3 nav_metrics.py -1.5 -1.5      # 指定目标
"""
import sys
import time

import rclpy
import numpy as np
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from nav_msgs.msg import Path
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy

INIT_POSE = (0.03, -0.84, -0.983, 0.184)

goal = (1.5, 1.5) if len(sys.argv) < 3 else (float(sys.argv[1]), float(sys.argv[2]))


def make_pose(nav, x, y):
    p = PoseStamped()
    p.header.frame_id = "map"
    p.header.stamp = nav.get_clock().now().to_msg()
    p.pose.position.x, p.pose.position.y, p.pose.position.z = x, y, 0.0
    p.pose.orientation.w = 1.0
    return p


def main():
    rclpy.init()
    nav = BasicNavigator()

    # 初始位姿（BEST_EFFORT）
    qos = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                     durability=DurabilityPolicy.VOLATILE,
                     history=HistoryPolicy.KEEP_LAST, depth=1)
    raw = rclpy.create_node("metric_node")
    pub = raw.create_publisher(PoseWithCovarianceStamped, "initialpose", qos)

    msg = PoseWithCovarianceStamped()
    msg.header.frame_id = "map"
    msg.header.stamp = raw.get_clock().now().to_msg()
    msg.pose.pose = make_pose(nav, *INIT_POSE[:2]).pose
    msg.pose.covariance[0] = msg.pose.covariance[7] = 0.25
    msg.pose.covariance[35] = 0.068
    for _ in range(5):
        pub.publish(msg)
        rclpy.spin_once(raw, timeout_sec=0.5)
        time.sleep(0.3)

    nav.waitUntilNav2Active()
    print("Nav2 已激活")

    # 订阅 /plan 记录全局路径
    plans = []
    raw.create_subscription(Path, "plan", lambda p: plans.append(p), 10)

    g = make_pose(nav, *goal)
    print(f"导航目标 ({goal[0]}, {goal[1]})")
    t0 = time.time()
    nav.goToPose(g)
    while not nav.isTaskComplete():
        rclpy.spin_once(raw, timeout_sec=0.1)
    dt = time.time() - t0

    result = nav.getResult()
    ok = result == TaskResult.SUCCEEDED

    # 最长规划路径长度
    if plans:
        best = max(plans, key=lambda p: len(p.poses))
        pts = np.array([[p.pose.position.x, p.pose.position.y] for p in best.poses])
        seg = np.linalg.norm(np.diff(pts, axis=0), axis=1).sum()
        print(f"Nav2 全局路径: {len(best.poses)} 个点 | {seg:.2f} m")
        # 保存路径点（供与手写 A* 叠加对比）
        np.save(f"/tmp/nav2_plan_{goal[0]}_{goal[1]}.npy", pts)
        print(f"路径点已保存: /tmp/nav2_plan_{goal[0]}_{goal[1]}.npy")
    else:
        seg = 0.0
        print("未捕获 /plan")

    print(f"到达: {'✅' if ok else '❌'} | 耗时 {dt:.1f}s")
    print(f"METRICS: reach={ok} time={dt:.1f} plan_m={seg:.2f}")

    raw.destroy_node()
    nav.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
