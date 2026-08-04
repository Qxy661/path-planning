#!/usr/bin/env python3
"""
Nav2 自主导航：设置初始位姿 → 发送目标点 → 监控到达

支持两种模式：
1. 单目标：    python3 nav_goal.py [x y [yaw]]
2. 多目标巡检： python3 nav_goal.py --patrol "x1,y1;x2,y2;x3,y3"

实测（2026-08-04）：
    (0.03,-0.84) → (1.5,1.5)   ✅ 到达
    (1.5,1.5) → (-1.5,-1.5)    ✅ 到达（对角线穿越迷宫）
"""
import argparse
import sys
import time

import rclpy
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy

INIT_POSE = (0.03, -0.84, -0.983, 0.184)  # 初始位姿估计 (x, y, qz, qw)


def make_pose(nav: BasicNavigator, x, y, qz=0.0, qw=1.0):
    p = PoseStamped()
    p.header.frame_id = "map"
    p.header.stamp = nav.get_clock().now().to_msg()
    p.pose.position.x = x
    p.pose.position.y = y
    p.pose.position.z = 0.0
    p.pose.orientation.z = qz
    p.pose.orientation.w = qw
    return p


def get_current_odom(raw_node):
    """从 /odom 读取小车当前位置作为初始位姿估计."""
    import math
    from nav_msgs.msg import Odometry
    pose = {}
    def cb(msg):
        pose["p"] = msg.pose.pose
    raw_node.create_subscription(Odometry, "odom", cb, 10)
    for _ in range(10):
        rclpy.spin_once(raw_node, timeout_sec=0.5)
        if "p" in pose:
            break
    if "p" not in pose:
        return INIT_POSE
    p = pose["p"]
    return (p.position.x, p.position.y, p.orientation.z, p.orientation.w)


def send_initial_pose(nav: BasicNavigator):
    """AMCL 以 BEST_EFFORT 订阅 /initialpose，须用相同 QoS 发布."""
    qos = QoSProfile(
        reliability=ReliabilityPolicy.BEST_EFFORT,
        durability=DurabilityPolicy.VOLATILE,
        history=HistoryPolicy.KEEP_LAST,
        depth=1)
    raw_node = rclpy.create_node("initial_pose_pub")
    pub = raw_node.create_publisher(PoseWithCovarianceStamped, "initialpose", qos)

    init = get_current_odom(raw_node)
    print(f"    ✓ 从 /odom 读取当前位置作为初始位姿: ({init[0]:.2f}, {init[1]:.2f})")

    msg = PoseWithCovarianceStamped()
    msg.header.frame_id = "map"
    msg.header.stamp = raw_node.get_clock().now().to_msg()
    msg.pose.pose = make_pose(nav, *init).pose
    msg.pose.covariance[0] = 0.25
    msg.pose.covariance[7] = 0.25
    msg.pose.covariance[35] = 0.068
    for _ in range(5):  # 多发几次确保送达
        pub.publish(msg)
        rclpy.spin_once(raw_node, timeout_sec=0.5)
        time.sleep(0.3)
    raw_node.destroy_node()


def navigate(nav: BasicNavigator, x, y):
    """发送单目标并阻塞监控，返回 (结果, 耗时秒)."""
    goal = make_pose(nav, x, y)
    print(f"    → 目标 ({x}, {y})")
    nav.goToPose(goal)
    t0 = time.time()
    while not nav.isTaskComplete():
        time.sleep(0.5)
    dt = time.time() - t0
    result = nav.getResult()
    status = {
        TaskResult.SUCCEEDED: "✅ 到达",
        TaskResult.CANCELED: "⚠️ 取消",
        TaskResult.FAILED: "❌ 失败",
    }.get(result, f"其他({result})")
    print(f"    {status}（耗时 {dt:.1f}s）")
    return result == TaskResult.SUCCEEDED, dt


def main():
    parser = argparse.ArgumentParser(description="Nav2 自主导航")
    parser.add_argument("goal", nargs="*", help="单目标: x y [yaw]")
    parser.add_argument("--patrol", metavar="\"x,y;x,y;...\"",
                        help="多目标巡检: 分号分隔坐标点，每点逗号分隔 xy")
    args = parser.parse_args()

    # 解析目标
    if args.patrol:
        waypoints = [tuple(map(float, w.split(","))) for w in args.patrol.split(";")]
        mode = "patrol"
    elif len(args.goal) >= 2:
        waypoints = [(float(args.goal[0]), float(args.goal[1]))]
        mode = "single"
    else:
        waypoints = [(1.5, 1.5)]
        mode = "single"

    rclpy.init()
    nav = BasicNavigator()

    print(f"[1] 设置初始位姿 ({INIT_POSE[0]}, {INIT_POSE[1]}) [BEST_EFFORT]")
    send_initial_pose(nav)
    print("    ✓ 已发布")

    print("[2] 等待 Nav2 激活...")
    nav.waitUntilNav2Active()
    print("    ✓ Nav2 已激活，定位完成")

    print(f"[3] 开始导航（{mode} 模式，{len(waypoints)} 个目标）")
    results = []
    for i, (x, y) in enumerate(waypoints, 1):
        print(f"  [{i}/{len(waypoints)}]")
        ok, dt = navigate(nav, x, y)
        results.append((x, y, ok, dt))
        if not ok:
            print("    该目标失败，继续下一目标")

    # 汇总
    succ = sum(1 for _, _, ok, _ in results if ok)
    total = sum(dt for _, _, _, dt in results)
    print(f"\n巡检完成: {succ}/{len(waypoints)} 目标到达，总耗时 {total:.1f}s")
    for x, y, ok, dt in results:
        print(f"    ({x}, {y}): {'✅' if ok else '❌'} {dt:.1f}s")

    nav.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
