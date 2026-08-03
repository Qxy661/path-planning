"""
RRT* 路径规划算法库（标准实现）

采样类全局规划器。随机采样探索空间，rewiring 达到渐近最优。
特点：高维/复杂环境适用，不保证最优但渐近最优（vs A* 的保证最优）。

Usage:
    from rrt_star import RRTStar
    planner = RRTStar(obstacles, bounds, step_size=0.5)
    path = planner.plan(start, goal)

原理：见 docs/02-全局规划(A*RRT).md
"""
import numpy as np


class Node:
    def __init__(self, pos):
        self.pos = np.array(pos, dtype=float)
        self.parent = None
        self.cost = 0.0


class RRTStar:
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

    def _random_pos(self):
        """随机采样（一定概率直接采终点，加速收敛）."""
        if np.random.random() < self.goal_sample_rate:
            return self.goal_pos.copy()
        xmin, xmax, ymin, ymax = self.bounds
        return np.array([
            np.random.uniform(xmin, xmax),
            np.random.uniform(ymin, ymax),
        ])

    def _nearest(self, pos):
        """找最近节点."""
        return min(self.nodes, key=lambda n: np.linalg.norm(n.pos - pos))

    def _near(self, pos, radius):
        """找半径内所有节点（用于 rewiring）."""
        return [n for n in self.nodes
                if np.linalg.norm(n.pos - pos) < radius]

    def _steer(self, from_pos, to_pos):
        """从 from 向 to 步进 step_size."""
        delta = to_pos - from_pos
        dist = np.linalg.norm(delta)
        if dist < self.step_size:
            return to_pos
        return from_pos + delta / dist * self.step_size

    def _collision_free(self, a, b):
        """检查 a->b 连线是否无障碍（插值采样）."""
        a, b = np.array(a, dtype=float), np.array(b, dtype=float)
        dist = np.linalg.norm(b - a)
        steps = int(dist / 0.1) + 1
        for i in range(1, steps + 1):
            p = a + (b - a) * i / steps
            if self.obstacle_check(p):
                return False
        return True

    def plan(self, start, goal, visualize=False):
        """RRT* 规划. 返回路径节点列表 或 None."""
        self.start_pos = np.array(start, dtype=float)
        self.goal_pos = np.array(goal, dtype=float)

        if not self._collision_free(self.start_pos, self.start_pos):
            return None
        self.nodes = [Node(self.start_pos)]

        for _ in range(self.max_iter):
            rand_pos = self._random_pos()
            nearest = self._nearest(rand_pos)
            new_pos = self._steer(nearest.pos, rand_pos)

            if self._collision_free(nearest.pos, new_pos):
                new_node = Node(new_pos)
                new_node.cost = nearest.cost + np.linalg.norm(new_pos - nearest.pos)
                new_node.parent = nearest

                # RRT* 核心：rewiring（在附近节点中找更优父节点）
                for near_node in self._near(new_pos, self.search_radius):
                    if self._collision_free(near_node.pos, new_pos):
                        new_cost = near_node.cost + np.linalg.norm(new_pos - near_node.pos)
                        if new_cost < new_node.cost:
                            new_node.cost = new_cost
                            new_node.parent = near_node

                self.nodes.append(new_node)

                # 反向 rewiring：以新节点为父节点，降低附近节点代价
                for near_node in self._near(new_pos, self.search_radius):
                    if self._collision_free(new_pos, near_node.pos):
                        new_cost = new_node.cost + np.linalg.norm(near_node.pos - new_pos)
                        if new_cost < near_node.cost:
                            near_node.cost = new_cost
                            near_node.parent = new_node

                # 到达终点附近
                if np.linalg.norm(new_pos - self.goal_pos) < self.step_size:
                    goal_node = Node(self.goal_pos)
                    goal_node.cost = new_node.cost + np.linalg.norm(self.goal_pos - new_pos)
                    goal_node.parent = new_node
                    return self._extract_path(goal_node)

        # 找离终点最近的节点
        if self.nodes:
            closest = min(self.nodes, key=lambda n: np.linalg.norm(n.pos - self.goal_pos))
            if np.linalg.norm(closest.pos - self.goal_pos) < 3.0:
                goal_node = Node(self.goal_pos)
                goal_node.cost = closest.cost + np.linalg.norm(self.goal_pos - closest.pos)
                goal_node.parent = closest
                return self._extract_path(goal_node)
        return None

    def _extract_path(self, goal_node):
        """回溯路径."""
        path = []
        current = goal_node
        while current is not None:
            path.append(current.pos)
            current = current.parent
        return path[::-1]
