"""
算法性能基准（A* vs RRT*）

量化对比：路径长度 / 节点数 / 耗时。
体现"评估→对比"的科研方法。

Usage:
    python exploration/compare_benchmark.py
"""
import sys
import os
import time
sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "algorithms"))

import numpy as np
from astar import AStar
from rrt_star import RRTStar


def astar_benchmark():
    """A* 基准（栅格地图）."""
    grid = np.zeros((20, 20), dtype=int)
    grid[5:15, 7] = 1
    grid[10, 3:7] = 1
    grid[2:18, 13] = 1

    planner = AStar(grid)
    t0 = time.time()
    path, visited = planner.plan((1, 1), (18, 18))
    dt = time.time() - t0
    return {
        "path_len": len(path) if path else 0,
        "nodes": visited,
        "time_ms": dt * 1000,
        "optimal": True,
    }


def rrt_benchmark():
    """RRT* 基准（连续空间）."""
    def check(pos):
        for ox, oy, r in [(5, 5, 1.0), (15, 10, 1.2), (8, 15, 1.0)]:
            if np.hypot(pos[0] - ox, pos[1] - oy) < r:
                return True
        return False

    planner = RRTStar(check, (0, 20, 0, 20), step_size=0.5, max_iter=1000)
    t0 = time.time()
    path = planner.plan((1.0, 1.0), (18.0, 18.0))
    dt = time.time() - t0
    return {
        "path_len": len(path) if path else 0,
        "nodes": len(planner.nodes),
        "time_ms": dt * 1000,
        "optimal": False,  # RRT* 渐近最优
    }


def main():
    print("=" * 50)
    print("  算法性能基准")
    print("=" * 50)

    print("\n=== A*（图搜索，栅格）===")
    a = astar_benchmark()
    print(f"  路径: {a['path_len']}步, 节点: {a['nodes']}, 耗时: {a['time_ms']:.1f}ms")
    print(f"  最优性: 保证最优")

    print("\n=== RRT*（采样，连续空间）===")
    r = rrt_benchmark()
    print(f"  路径: {r['path_len']}步, 节点: {r['nodes']}, 耗时: {r['time_ms']:.1f}ms")
    print(f"  最优性: 渐近最优")

    # 输出 markdown 表格（供 results/comparison.md）
    print("\n=== Markdown 表格 ===")
    print("| 算法 | 路径 | 节点 | 耗时 | 最优性 |")
    print("|---|---|---|---|---|")
    print(f"| A* | {a['path_len']}步 | {a['nodes']} | {a['time_ms']:.1f}ms | 最优 |")
    print(f"| RRT* | {r['path_len']}步 | {r['nodes']} | {r['time_ms']:.1f}ms | 渐近最优 |")


if __name__ == "__main__":
    main()
