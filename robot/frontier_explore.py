#!/usr/bin/env python3
"""
前沿探索自主建图（frontier-based exploration）

机器人自动检测「空闲格↔未知格」边界（前沿），贪心选择最近前沿
导航过去，循环直到地图被完全探索——从空白地图自动生长到完整覆盖。

原理：
  前沿单元 = 空闲(0) 且 4/8 邻域含未知(-1) 的栅格
  连通域聚类 → 前沿区域 → 目标 = 最近前沿质心

架构要求（SLAM 模式）：
  - Cartographer 运行（live SLAM，发布 /map + map→odom）
  - Nav2 navigation_launch.py（无 map_server/amcl，costmap 用 SLAM 的 /map）

用法：
    python3 frontier_explore.py
"""
import math
import time
from collections import deque

import numpy as np
import rclpy
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import OccupancyGrid
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, HistoryPolicy, ReliabilityPolicy
from rclpy.time import Time
import tf2_ros


class FrontierExplorer(Node):
    def __init__(self):
        super().__init__("frontier_explorer")
        # /map 订阅（Cartographer 实时发布）
        map_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1)
        self.create_subscription(OccupancyGrid, "map", self.map_cb, map_qos)
        self.map = None

        # 机器人位姿：TF map→base_link（Cartographer 在 SLAM 模式提供）
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # 导航动作客户端
        self.nav = ActionClient(self, NavigateToPose, "navigate_to_pose")
        self.get_logger().info("前沿探索启动：等待 /map 与导航服务...")

    def map_cb(self, msg):
        """保存最新地图."""
        self.map = msg

    def wait_for_ready(self):
        """等待地图和导航服务就绪."""
        while rclpy.ok() and (self.map is None or not self.nav.wait_for_server(timeout_sec=1.0)):
            rclpy.spin_once(self, timeout_sec=0.2)
        self.get_logger().info("就绪：地图 + 导航服务已连接")

    def detect_frontiers(self):
        """检测前沿单元（空闲格邻接未知格）→ 返回前沿栅格坐标列表.

        Cartographer 的 /map 值约定：未知=-1，空闲=低概率(通常0-49)，占用=高概率(50-100)。
        空闲阈值取 50（与 Nav2 占用阈值一致）。
        """
        m = self.map
        w, h = m.info.width, m.info.height
        data = np.array(m.data, dtype=np.int8).reshape(h, w)

        free = (data >= 0) & (data < 50)
        unknown = data < 0

        # 前沿 = 空闲 且 4 邻域含未知
        frontier = np.zeros((h, w), dtype=bool)
        for dy, dx in [(-1, 0), (1, 0), (0, -1), (0, 1)]:
            shifted = np.roll(unknown, (dy, dx), axis=(0, 1))
            frontier |= free & shifted
        # 去掉边缘（roll 引入的环绕）
        frontier[0, :] = frontier[-1, :] = False
        frontier[:, 0] = frontier[:, -1] = False

        ys, xs = np.nonzero(frontier)
        return list(zip(xs, ys))

    def cluster_frontiers(self, cells):
        """连通域聚类（BFS）→ 返回前沿区域列表（每个是栅格坐标列表）."""
        if not cells:
            return []
        cell_set = set(cells)
        visited = set()
        clusters = []
        for cell in cells:
            if cell in visited:
                continue
            # BFS
            q = deque([cell])
            visited.add(cell)
            cluster = []
            while q:
                c = q.popleft()
                cluster.append(c)
                cx, cy = c
                for dx, dy in [(-1, 0), (1, 0), (0, -1), (0, 1), (-1, -1), (1, 1), (-1, 1), (1, -1)]:
                    n = (cx + dx, cy + dy)
                    if n in cell_set and n not in visited:
                        visited.add(n)
                        q.append(n)
            clusters.append(cluster)
        return clusters

    def cluster_centroid_world(self, cluster):
        """前沿区域质心（世界坐标）——质心偏移到已知空闲侧，确保可达."""
        m = self.map
        w, h = m.info.width, m.info.height
        data = np.array(m.data, dtype=np.int8).reshape(h, w)

        xs = [c[0] for c in cluster]
        ys = [c[1] for c in cluster]
        cx = int(sum(xs) / len(xs))
        cy = int(sum(ys) / len(ys))
        # 若质心不在空闲区，沿 4 邻域找最近的空闲格（保证目标可达）
        if not (0 <= cy < h and 0 <= cx < w and 0 <= data[cy, cx] < 50):
            for d in range(1, 15):
                found = False
                for dx in range(-d, d + 1):
                    for dy in range(-d, d + 1):
                        nx, ny = cx + dx, cy + dy
                        if 0 <= ny < h and 0 <= nx < w and 0 <= data[ny, nx] < 50:
                            cx, cy, found = nx, ny, True
                            break
                    if found:
                        break
                if found:
                    break
        wx = m.info.origin.position.x + cx * m.info.resolution
        wy = m.info.origin.position.y + cy * m.info.resolution
        return (wx, wy, len(cluster))

    def current_pose(self):
        """当前位姿：从 TF map→base_link 获取（最新可用变换）."""
        try:
            tf = self.tf_buffer.lookup_transform(
                "map", "base_link", rclpy.time.Time(),
                rclpy.duration.Duration(seconds=0.5))
            return (tf.transform.translation.x, tf.transform.translation.y)
        except Exception:
            return None

    def send_goal(self, x, y):
        """发送 NavigateToPose 目标并阻塞等待到达."""
        goal = NavigateToPose.Goal()
        goal.pose.header.frame_id = "map"
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        goal.pose.pose.position.x = x
        goal.pose.pose.position.y = y
        goal.pose.pose.orientation.w = 1.0

        self.get_logger().info(f"  导航到前沿 ({x:.2f}, {y:.2f})")
        # 手动轮询：处理回调 + 带超时（避免目标不可达时死等）
        future = self.nav.send_goal_async(goal)
        deadline = time.time() + 10.0
        while rclpy.ok() and time.time() < deadline and not future.done():
            rclpy.spin_once(self, timeout_sec=0.2)
        if not future.done() or not future.result():
            self.get_logger().warn("  目标未接受，放弃该前沿")
            return False
        gh = future.result()
        result_future = gh.get_result_async()
        deadline = time.time() + 90.0
        while rclpy.ok() and time.time() < deadline and not result_future.done():
            rclpy.spin_once(self, timeout_sec=0.2)
        if not result_future.done():
            self.get_logger().warn("  导航超时(90s)，取消该前沿")
            gh.cancel_goal()  # 取消，让机器人停下继续探索
            return False
        return result_future.result().status == GoalStatus.STATUS_SUCCEEDED

    def coverage(self):
        """当前覆盖率 = (空闲+占用) / (总 - 越界未知)."""
        m = self.map
        data = np.array(m.data, dtype=np.int8)
        known = data >= 0
        return float(known.sum()) / len(data)

    def explore(self, max_rounds=20, log_cov="/tmp/frontier_cov.csv"):
        """主循环：检测前沿 → 贪心选择 → 导航 → 重复."""
        import csv
        cov_f = open(log_cov, "w")
        cov_w = csv.writer(cov_f)
        cov_w.writerow(["round", "coverage", "elapsed_s"])
        t0 = time.time()

        for round_no in range(1, max_rounds + 1):
            cells = self.detect_frontiers()
            if not cells:
                self.get_logger().info("🎉 无剩余前沿，建图完成！")
                return True

            clusters = self.cluster_frontiers(cells)
            targets = [self.cluster_centroid_world(c) for c in clusters if len(c) > 3]
            if not targets:
                self.get_logger().info("🎉 无可探索前沿，建图完成！")
                return True

            # 贪心：选最近的（加权：距离 / 区域大小）
            pose = self.current_pose()
            if pose:
                targets.sort(key=lambda t: math.dist(t[:2], pose) - t[2] * 0.01)
            else:
                targets.sort(key=lambda t: -t[2])

            wx, wy, _ = targets[0]
            cov = self.coverage()
            cov_w.writerow([round_no, round(cov, 4), round(time.time() - t0, 1)])
            cov_f.flush()
            self.get_logger().info(f"[{round_no}] 前沿区域 {len(clusters)} 个 | 覆盖率 {cov:.1%} | 目标 ({wx:.2f}, {wy:.2f})")
            ok = self.send_goal(wx, wy)
            if not ok:
                self.get_logger().warn("  目标未到达，重选下一前沿")
            rclpy.spin_once(self, timeout_sec=1.0)

        cov_f.close()

        self.get_logger().info("达到最大轮次，停止")
        return False


def main():
    rclpy.init()
    node = FrontierExplorer()
    try:
        node.wait_for_ready()
        node.explore()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
