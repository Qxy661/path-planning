#!/usr/bin/env python3
"""
局部规划器对比：采集导航轨迹 + 计算量化指标

用法：
    python3 planner_compare.py <标签> [目标x] [目标y]
    例： python3 planner_compare.py dwb
        python3 planner_compare.py mppi 1.5 1.5

指标：
    - 到达时间 (s)
    - 轨迹长度 (m)
    - 平滑度（平均转向角/曲率）
    - 是否到达
输出：
    - /tmp/traj_<标签>.npy  轨迹点
    - /tmp/metrics_<标签>.json 指标
"""
import json
import sys
import time

import numpy as np
import rclpy
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import Odometry
from rclpy.action import ActionClient
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy

label = sys.argv[1] if len(sys.argv) > 1 else "planner"
goal = (1.5, 1.5) if len(sys.argv) < 4 else (float(sys.argv[2]), float(sys.argv[3]))


def main():
    rclpy.init()
    raw = rclpy.create_node("traj_recorder")

    # 初始位姿（BEST_EFFORT，兼容 AMCL）
    qos = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                     durability=DurabilityPolicy.VOLATILE,
                     history=HistoryPolicy.KEEP_LAST, depth=1)
    pub = raw.create_publisher(PoseWithCovarianceStamped, "initialpose", qos)

    # 从 odom 读当前位置
    pose = {}
    def odom_cb(m):
        pose["p"] = m.pose.pose
    raw.create_subscription(Odometry, "odom", odom_cb, 10)
    for _ in range(10):
        rclpy.spin_once(raw, timeout_sec=0.5)
        if "p" in pose:
            break
    init = pose.get("p")

    msg = PoseWithCovarianceStamped()
    msg.header.frame_id = "map"
    msg.header.stamp = raw.get_clock().now().to_msg()
    if init:
        msg.pose.pose.position.x = init.position.x
        msg.pose.pose.position.y = init.position.y
        msg.pose.pose.orientation = init.orientation
    else:
        msg.pose.pose.orientation.w = 1.0
    # 收紧协方差（迷宫对称环境防止 AMCL 收敛到错误位置）
    msg.pose.covariance[0] = msg.pose.covariance[7] = 0.01
    msg.pose.covariance[35] = 0.01
    for _ in range(5):
        pub.publish(msg)
        rclpy.spin_once(raw, timeout_sec=0.5)
        time.sleep(0.3)

    # 等待 navigate_to_pose 服务（SLAM 模式无 AMCL，不依赖 waitUntilNav2Active）
    nav_client = ActionClient(raw, NavigateToPose, "navigate_to_pose")
    while rclpy.ok() and not nav_client.wait_for_server(timeout_sec=1.0):
        rclpy.spin_once(raw, timeout_sec=0.2)
    print(f"[{label}] 导航服务就绪，目标 {goal}")

    # 记录轨迹
    traj = []
    def traj_cb(m):
        traj.append([m.pose.pose.position.x, m.pose.pose.position.y])
    raw.create_subscription(Odometry, "odom", traj_cb, 10)

    # 发送导航目标
    gmsg = NavigateToPose.Goal()
    gmsg.pose.header.frame_id = "map"
    gmsg.pose.header.stamp = raw.get_clock().now().to_msg()
    gmsg.pose.pose.position.x, gmsg.pose.pose.position.y = goal
    gmsg.pose.pose.orientation.w = 1.0

    t0 = time.time()
    goal_future = nav_client.send_goal_async(gmsg)
    rclpy.spin_until_future_complete(raw, goal_future, timeout_sec=10.0)
    if not goal_future.done() or not goal_future.result():
        ok = False
        dt = time.time() - t0
    else:
        gh = goal_future.result()
        result_future = gh.get_result_async()
        rclpy.spin_until_future_complete(raw, result_future, timeout_sec=90.0)
        dt = time.time() - t0
        if result_future.done():
            ok = result_future.result().status == GoalStatus.STATUS_SUCCEEDED
        else:
            gh.cancel_goal()
            ok = False

    # 指标
    pts = np.array(traj) if traj else np.zeros((0, 2))
    length = 0.0
    if len(pts) > 1:
        length = np.linalg.norm(np.diff(pts, axis=0), axis=1).sum()
    # 平滑度：相邻段方向角变化的均值
    smoothness = float("nan")
    if len(pts) > 3:
        seg = np.diff(pts, axis=0)
        ang = np.arctan2(seg[:, 1], seg[:, 0])
        d_ang = np.abs(np.diff(ang))
        smoothness = float(np.mean(d_ang))

    metrics = {
        "label": label, "goal": list(goal), "reach": ok,
        "time_s": round(dt, 2), "length_m": round(length, 2),
        "smoothness_rad": round(smoothness, 4) if not np.isnan(smoothness) else None,
        "samples": len(pts),
    }
    np.save(f"/tmp/traj_{label}.npy", pts)
    with open(f"/tmp/metrics_{label}.json", "w") as f:
        json.dump(metrics, f, indent=2, ensure_ascii=False)
    print(json.dumps(metrics, ensure_ascii=False))

    raw.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
