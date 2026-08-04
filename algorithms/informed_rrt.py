"""
Informed RRT* 路径规划算法库（前沿增强）

RRT* 的最优采样变体：找到初始路径后，只在"最优路径的椭圆区域"采样，
加速收敛到最优解（比 RRT* 更快、更优）。

Usage:
    from informed_rrt import InformedRRT
    planner = InformedRRT(obstacle_check, bounds)
    path = planner.plan(start, goal)

原理：见 docs/02-全局规划算法.md
"""
import math
import numpy as np


class Node:
    def __init__(self, pos):
        self.pos = np.array(pos, dtype=float)
        self.parent = None
        self.cost = 0.0


class InformedRRT:
    def __init__(self, obstacle_check, bounds, step_size=0.5,
                 max_iter=2000, goal_sample_rate=0.1, search_radius=1.5):
        """初始化.
        obstacle_check: 函数(pos) -> True如果障碍物
        bounds: (xmin, xmax, ymin, ymax)
        """
        self.obstacle_check = obstacle_check
        self.bounds = bounds
        self.step_size = step_size
        self.max_iter = max_iter
        self.goal_sample_rate = goal_sample_rate
        self.search_radius = search_radius
        self.nodes = []
        self.best_path = None
        self.best_path_cost = float("inf")
        self.ellipse_c = None  # 椭圆中心到焦点距离
        self.ellipse_min_dist = 0.0  # 椭圆短轴

    def _random_pos(self):
        """采样：有最优路径时在椭圆内采样（Informed 核心）."""
        # 一定概率直接采终点
        if np.random.random() < self.goal_sample_rate:
            return self.goal_pos.copy()
        # 有最优路径 → 椭圆采样
        if self.best_path is not None and self.ellipse_c > 0:
            return self._sample_ellipse()
        # 否则均匀采样
        xmin, xmax, ymin, ymax = self.bounds
        return np.array([
            np.random.uniform(xmin, xmax),
            np.random.uniform(ymin, ymax),
        ])

    def _sample_ellipse(self):
        """在椭圆区域内采样（Informed RRT* 核心优化）."""
        # 椭圆参数：c = 焦点距/2, a = 最优路径/2, b = sqrt(a^2 - c^2)
        c = self.ellipse_c
        a = self.best_path_cost / 2.0
        b = math.sqrt(max(a * a - c * c, 1e-6))

        # 在单位圆采样
        theta = np.random.uniform(0, 2 * math.pi)
        r = math.sqrt(np.random.random())  # 均匀分布
        x = a * r * math.cos(theta)
        y = b * r * math.sin(theta)

        # 旋转到焦点方向
        start = self.start_pos
        goal = self.goal_pos
        dx, dy = goal - start
        angle = math.atan2(dy, dx)
        rot = np.array([[math.cos(angle), -math.sin(angle)],
                        [math.sin(angle), math.cos(angle)]])
        center = (start + goal) / 2.0
        return center + rot @ np.array([x, y])

    def _nearest(self, pos):
        return min(self.nodes, key=lambda n: np.linalg.norm(n.pos - pos))

    def _near(self, pos, radius):
        return [n for n in self.nodes
                if np.linalg.norm(n.pos - pos) < radius]

    def _steer(self, from_pos, to_pos):
        delta = to_pos - from_pos
        dist = np.linalg.norm(delta)
        if dist < self.step_size:
            return to_pos
        return from_pos + delta / dist * self.step_size

    def _collision_free(self, a, b):
        a, b = np.array(a, dtype=float), np.array(b, dtype=float)
        dist = np.linalg.norm(b - a)
        steps = int(dist / 0.1) + 1
        for i in range(1, steps + 1):
            p = a + (b - a) * i / steps
            if self.obstacle_check(p):
                return False
        return True

    def plan(self, start, goal):
        """Informed RRT* 规划. 返回路径节点列表 或 None."""
        self.start_pos = np.array(start, dtype=float)
        self.goal_pos = np.array(goal, dtype=float)
        self.ellipse_c = np.linalg.norm(self.goal_pos - self.start_pos) / 2.0

        self.nodes = [Node(self.start_pos)]

        for _ in range(self.max_iter):
            rand_pos = self._random_pos()
            nearest = self._nearest(rand_pos)
            new_pos = self._steer(nearest.pos, rand_pos)

            if not self._collision_free(nearest.pos, new_pos):
                continue

            new_node = Node(new_pos)
            new_node.cost = nearest.cost + np.linalg.norm(new_pos - nearest.pos)
            new_node.parent = nearest

            # rewiring（同 RRT*）
            for near_node in self._near(new_pos, self.search_radius):
                if self._collision_free(near_node.pos, new_pos):
                    new_cost = near_node.cost + np.linalg.norm(new_pos - near_node.pos)
                    if new_cost < new_node.cost:
                        new_node.cost = new_cost
                        new_node.parent = near_node

            self.nodes.append(new_node)

            # 反向 rewiring
            for near_node in self._near(new_pos, self.search_radius):
                if self._collision_free(new_pos, near_node.pos):
                    new_cost = new_node.cost + np.linalg.norm(near_node.pos - new_pos)
                    if new_cost < near_node.cost:
                        near_node.cost = new_cost
                        near_node.parent = new_node

            # 更新最优路径（Informed 核心：到目标且更优）
            if np.linalg.norm(new_pos - self.goal_pos) < self.step_size:
                path_cost = new_node.cost + np.linalg.norm(self.goal_pos - new_pos)
                if path_cost < self.best_path_cost:
                    self.best_path_cost = path_cost
                    goal_node = Node(self.goal_pos)
                    goal_node.cost = path_cost
                    goal_node.parent = new_node
                    self.best_path = self._extract_path(goal_node)

        return self.best_path

    def _extract_path(self, goal_node):
        path = []
        current = goal_node
        while current is not None:
            path.append(current.pos)
            current = current.parent
        return path[::-1]
