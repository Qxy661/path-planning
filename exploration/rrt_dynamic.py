"""
RRT* + 动态障碍场景（探索应用）

高设计度：多场景障碍变化，验证 RRT* 适应性。
学习式思路：对比不同障碍布局下 RRT* 的表现（节点数/路径）。

Usage:
    python exploration/rrt_dynamic.py
"""
import sys
import os
sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "algorithms"))

import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from rrt_star import RRTStar


def obstacle_check(pos, obstacles):
    for ox, oy, r in obstacles:
        if np.hypot(pos[0] - ox, pos[1] - oy) < r:
            return True
    return False


def run_scenario(obstacles, start, goal):
    """运行 RRT*，返回 (路径, 节点数)."""
    check = lambda p: obstacle_check(p, obstacles)
    planner = RRTStar(check, (0, 20, 0, 20),
                      step_size=0.5, max_iter=1200)
    path = planner.plan(start, goal)
    return path, len(planner.nodes)


def main():
    # 动态障碍场景（复杂度递增）
    scenarios = [
        ("简单", [(5, 5, 1.0), (15, 10, 1.2)]),
        ("中等", [(6, 4, 1.0), (12, 12, 1.0), (16, 6, 0.8)]),
        ("复杂", [(4, 8, 1.2), (10, 5, 0.8), (15, 14, 1.0), (8, 16, 1.0)]),
    ]
    start = (1.0, 1.0)
    goal = (18.0, 18.0)

    fig, axes = plt.subplots(1, 3, figsize=(15, 5))
    for i, (name, obs) in enumerate(scenarios):
        path, nodes = run_scenario(obs, start, goal)
        ax = axes[i]
        for ox, oy, r in obs:
            ax.add_patch(plt.Circle((ox, oy), r, color="gray", alpha=0.5))
        if path:
            p = np.array(path)
            ax.plot(p[:, 0], p[:, 1], "r-", linewidth=2, label="RRT* path")
        ax.plot(start[0], start[1], "go", markersize=8)
        ax.plot(goal[0], goal[1], "bo", markersize=8)
        ax.set_title(f"{name}: {nodes} nodes, path {len(path) if path else 0}")
        ax.set_xlim(0, 20)
        ax.set_ylim(0, 20)
        ax.set_aspect("equal")
        ax.legend(fontsize=8)

    plt.tight_layout()
    os.makedirs("results", exist_ok=True)
    plt.savefig("results/rrt_dynamic.png", dpi=120)
    print("已保存 results/rrt_dynamic.png")


if __name__ == "__main__":
    main()
