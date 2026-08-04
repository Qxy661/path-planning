#!/usr/bin/env python3
"""
Nav2 自主导航验证：设置初始位姿 → 发送目标点 → 监控到达

用法:
    python3 nav_goal.py                          # 默认目标 (1.5, 1.5)
    python3 nav_goal.py -1.5 -1.5                # 指定目标 x y
    python3 nav_goal.py 1.5 1.5 0.0              # 指定目标 x y yaw
"""
import sys
import time

import rclpy
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy

# 参数
init_pose = (0.03, -0.84, -0.983, 0.184)  # 初始位姿估计 (x, y, qz, qw)
goal_pose = (1.5, 1.5, 0.0, 1.0)          # 目标 (x, y, qz, qw)

if len(sys.argv) >= 3:
    gx, gy = float(sys.argv[1]), float(sys.argv[2])
    gz = float(sys.argv[3]) if len(sys.argv) > 3 else 0.0
    goal_pose = (gx, gy, gz, 1.0)


def make_pose(nav: BasicNavigator, x, y, qz, qw):
    p = PoseStamped()
    p.header.frame_id = "map"
    p.header.stamp = nav.get_clock().now().to_msg()
    p.pose.position.x = x
    p.pose.position.y = y
    p.pose.position.z = 0.0
    p.pose.orientation.z = qz
    p.pose.orientation.w = qw
    return p


def main():
    rclpy.init()
    nav = BasicNavigator()

    # AMCL 以 BEST_EFFORT 订阅 /initialpose，须用相同 QoS 发布
    qos = QoSProfile(
        reliability=ReliabilityPolicy.BEST_EFFORT,
        durability=DurabilityPolicy.VOLATILE,
        history=HistoryPolicy.KEEP_LAST,
        depth=1)
    raw_node = rclpy.create_node("initial_pose_pub")
    pub = raw_node.create_publisher(PoseWithCovarianceStamped, "initialpose", qos)

    print(f"[1/3] 设置初始位姿 ({init_pose[0]}, {init_pose[1]}) [BEST_EFFORT]")
    msg = PoseWithCovarianceStamped()
    msg.header.frame_id = "map"
    msg.header.stamp = raw_node.get_clock().now().to_msg()
    msg.pose.pose = make_pose(nav, *init_pose).pose
    msg.pose.covariance[0] = 0.25
    msg.pose.covariance[7] = 0.25
    msg.pose.covariance[35] = 0.068
    for _ in range(5):  # 多发几次确保送达
        pub.publish(msg)
        rclpy.spin_once(raw_node, timeout_sec=0.5)
        time.sleep(0.3)
    print("    ✓ 初始位姿已发布")

    print("[2/3] 等待 Nav2 激活（定位 + 生命周期）...")
    nav.waitUntilNav2Active()  # 阻塞直到 AMCL 就绪
    print("    ✓ Nav2 已激活，定位完成")

    goal = make_pose(nav, *goal_pose)
    print(f"[3/3] 发送导航目标 ({goal_pose[0]}, {goal_pose[1]})")
    nav.goToPose(goal)

    # 监控进度
    while not nav.isTaskComplete():
        feedback = nav.getFeedback()
        if feedback:
            dx = feedback.current_pose.pose.position.x
            dy = feedback.current_pose.pose.position.y
            print(f"    导航中... 当前位置 ({dx:.2f}, {dy:.2f})")
        time.sleep(1.0)

    result = nav.getResult()
    if result == TaskResult.SUCCEEDED:
        print("✅ 导航成功：小车到达目标点！")
    elif result == TaskResult.CANCELED:
        print("⚠️ 导航被取消")
    elif result == TaskResult.FAILED:
        print("❌ 导航失败（规划/控制错误，尝试其他目标点）")
    else:
        print(f"其他结果: {result}")

    nav.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
