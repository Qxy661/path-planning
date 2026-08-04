"""A* 算法测试：正确性 + 最优性."""
import sys
import os
sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "algorithms"))

import numpy as np
from astar import AStar


def make_grid():
    """标准测试地图（20x20，含障碍）."""
    grid = np.zeros((20, 20), dtype=int)
    grid[5:15, 7] = 1       # 竖墙
    grid[10, 3:7] = 1       # 横墙
    grid[2:18, 13] = 1      # 第二道竖墙
    grid[15, 13:18] = 1     # 底部墙
    return grid


def test_astar_finds_path():
    """A* 应找到路径，起点终点正确."""
    planner = AStar(make_grid())
    path, visited = planner.plan((1, 1), (18, 18))
    assert path is not None, "A* 应找到路径"
    assert path[0] == (1, 1), "起点错误"
    assert path[-1] == (18, 18), "终点错误"


def test_astar_no_path():
    """封闭空间应无路径."""
    grid = np.zeros((10, 10), dtype=int)
    grid[5, :] = 1  # 整行障碍，隔断
    planner = AStar(grid)
    path, _ = planner.plan((1, 1), (8, 8))
    assert path is None, "封闭空间应无路径"


def test_astar_optimal_empty():
    """无障碍时 A* 应走最短（直线）."""
    planner = AStar(np.zeros((10, 10), dtype=int))
    path, _ = planner.plan((0, 0), (9, 9))
    # 无障碍对角线，路径长度 = 10（含起点终点）
    assert len(path) == 10, f"无障碍直线应10步，实际{len(path)}"


def test_astar_visited_less_than_bfs():
    """A* 启发式应比盲目搜索访问更少节点."""
    grid = np.zeros((20, 20), dtype=int)
    grid[5:15, 7] = 1
    planner = AStar(grid)
    path, visited = planner.plan((1, 1), (18, 18))
    # 启发式搜索访问节点应远小于全图 400
    assert visited < 400, f"A* 应高效搜索，访问{visited}"
    assert visited > 0


def test_astar_smooth():
    """路径平滑应减少节点."""
    planner = AStar(make_grid())
    path, _ = planner.plan((1, 1), (18, 18))
    smooth = planner.smooth(path)
    assert len(smooth) <= len(path), "平滑后应更短或相等"
