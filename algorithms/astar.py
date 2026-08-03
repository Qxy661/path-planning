"""
A* 路径规划算法库（标准实现）

图搜索类全局规划器。在栅格地图上找最优路径。
f(n) = g(n) + h(n)：g=起点到n实际代价，h=n到终点估计代价（启发式）

Usage:
    from astar import AStar
    planner = AStar(grid)          # grid: 0空地 1障碍
    path = planner.plan(start, goal)  # [(x,y),...]

原理：见 docs/02-全局规划(A*RRT).md
"""
import heapq


class AStar:
    def __init__(self, grid, allow_diagonal=True):
        """初始化.
        grid: 2D numpy array, 0=free, 1=obstacle
        """
        self.grid = grid
        self.rows, self.cols = grid.shape
        self.allow_diagonal = allow_diagonal
        self._directions = self._get_directions()

    def _get_directions(self):
        """8邻域(含对角)或4邻域."""
        base = [(0, 1), (1, 0), (0, -1), (-1, 0)]
        if self.allow_diagonal:
            base += [(1, 1), (1, -1), (-1, 1), (-1, -1)]
        return base

    def _heuristic(self, a, b):
        """启发式：曼哈顿距离（4邻域）或欧氏距离（8邻域）."""
        if self.allow_diagonal:
            return ((a[0] - b[0]) ** 2 + (a[1] - b[1]) ** 2) ** 0.5
        return abs(a[0] - b[0]) + abs(a[1] - b[1])

    def _is_free(self, node):
        r, c = node
        if not (0 <= r < self.rows and 0 <= c < self.cols):
            return False
        return self.grid[r, c] == 0

    def plan(self, start, goal):
        """A* 搜索. 返回路径列表 或 None（无路径）.
        附带 visited_count 统计（体现启发式效率）.
        """
        if not self._is_free(start) or not self._is_free(goal):
            return None, 0

        open_heap = [(0, start)]  # (f, node)
        came_from = {}
        g_score = {start: 0}
        visited = 0

        while open_heap:
            _, current = heapq.heappop(open_heap)
            visited += 1

            if current == goal:
                path = []
                while current in came_from:
                    path.append(current)
                    current = came_from[current]
                path.append(start)
                return path[::-1], visited

            for dr, dc in self._directions:
                nr, nc = current[0] + dr, current[1] + dc
                neighbor = (nr, nc)
                if not self._is_free(neighbor):
                    continue
                # 对角不能穿墙
                if dr != 0 and dc != 0:
                    if not self._is_free((current[0] + dr, current[1])) or \
                       not self._is_free((current[0], current[1] + dc)):
                        continue
                step_cost = 1.0 if dr == 0 or dc == 0 else 1.414
                tentative_g = g_score[current] + step_cost

                if tentative_g < g_score.get(neighbor, float("inf")):
                    came_from[neighbor] = current
                    g_score[neighbor] = tentative_g
                    f = tentative_g + self._heuristic(neighbor, goal)
                    heapq.heappush(open_heap, (f, neighbor))

        return None, visited

    def smooth(self, path):
        """路径平滑：去除冗余拐点（视线可达判断）."""
        if path is None or len(path) < 3:
            return path
        smooth = [path[0]]
        for i in range(1, len(path) - 1):
            # 检查 smooth[-1] 到 path[i+1] 是否视线可达
            if not self._line_of_sight(smooth[-1], path[i + 1]):
                smooth.append(path[i])
        smooth.append(path[-1])
        return smooth

    def _line_of_sight(self, a, b):
        """Bresenham 直线可达判断."""
        x0, y0 = a
        x1, y1 = b
        dx, dy = abs(x1 - x0), abs(y1 - y0)
        sx = 1 if x0 < x1 else -1
        sy = 1 if y0 < y1 else -1
        err = dx - dy
        while True:
            if not self._is_free((x0, y0)):
                return False
            if x0 == x1 and y0 == y1:
                return True
            e2 = 2 * err
            if e2 > -dy:
                err -= dy
                x0 += sx
            if e2 < dx:
                err += dx
                y0 += sy
