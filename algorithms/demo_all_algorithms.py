"""
全算法对比演示：A* / RRT* / Informed RRT* / DWA

四格对比，直观展示各算法效果（作品集展示）。

Usage:
    python algorithms/demo_all_algorithms.py
"""
import sys
import os
sys.path.insert(0, os.path.dirname(__file__))

import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

from astar import AStar
from rrt_star import RRTStar
from informed_rrt import InformedRRT
from dwa import DWA


def make_grid():
    grid = np.zeros((20, 20), dtype=int)
    grid[5:15, 7] = 1
    grid[10, 3:7] = 1
    grid[2:18, 13] = 1
    grid[15, 13:18] = 1
    return grid


def make_obstacle_check():
    obstacles = [(5, 5, 1.0), (15, 10, 1.2), (8, 15, 1.0)]

    def check(pos):
        for ox, oy, r in obstacles:
            if np.hypot(pos[0] - ox, pos[1] - oy) < r:
                return True
        return False

    return check, obstacles


def demo_astar(ax):
    """A* 演示."""
    grid = make_grid()
    planner = AStar(grid)
    path, visited = planner.plan((1, 1), (18, 18))
    smooth = planner.smooth(path)

    ax.imshow(grid, cmap="Greys", origin="upper")
    if path:
        p = np.array(path)
        ax.plot(p[:, 1], p[:, 0], "b-", linewidth=2, label="A*")
        sp = np.array(smooth)
        ax.plot(sp[:, 1], sp[:, 0], "r--", linewidth=2, label="smoothed")
    ax.plot(1, 1, "go", markersize=10)
    ax.plot(18, 18, "bo", markersize=10)
    ax.set_title(f"A* ({visited} nodes)")
    ax.legend(fontsize=8)


def demo_rrt(ax, obstacles):
    """RRT* 演示."""
    check = lambda p: any(np.hypot(p[0]-ox, p[1]-oy) < r for ox, oy, r in obstacles)
    planner = RRTStar(check, (0, 20, 0, 20), step_size=0.5, max_iter=1000)
    path = planner.plan((1, 1), (18, 18))

    for ox, oy, r in obstacles:
        ax.add_patch(plt.Circle((ox, oy), r, color="gray", alpha=0.5))
    if path:
        p = np.array(path)
        ax.plot(p[:, 0], p[:, 1], "r-", linewidth=2)
    ax.plot(1, 1, "go", markersize=10)
    ax.plot(18, 18, "bo", markersize=10)
    ax.set_title(f"RRT* ({len(planner.nodes)} nodes)")
    ax.set_xlim(0, 20); ax.set_ylim(0, 20); ax.set_aspect("equal")


def demo_informed(ax, obstacles):
    """Informed RRT* 演示."""
    check = lambda p: any(np.hypot(p[0]-ox, p[1]-oy) < r for ox, oy, r in obstacles)
    planner = InformedRRT(check, (0, 20, 0, 20), step_size=0.5, max_iter=1000)
    path = planner.plan((1, 1), (18, 18))

    for ox, oy, r in obstacles:
        ax.add_patch(plt.Circle((ox, oy), r, color="gray", alpha=0.5))
    if path:
        p = np.array(path)
        ax.plot(p[:, 0], p[:, 1], "g-", linewidth=2)
    ax.plot(1, 1, "go", markersize=10)
    ax.plot(18, 18, "bo", markersize=10)
    cost = sum(np.linalg.norm(path[i+1]-path[i]) for i in range(len(path)-1)) if path else 0
    ax.set_title(f"Informed RRT* ({len(planner.nodes)} nodes, {cost:.1f})")
    ax.set_xlim(0, 20); ax.set_ylim(0, 20); ax.set_aspect("equal")


def demo_dwa(ax):
    """DWA 演示."""
    np.random.seed(42)
    obstacles = [(5, 5), (6, 5.5), (15, 10), (15.5, 10.5), (8, 15)]
    controller = DWA()
    state = np.array([1.0, 1.0, 0.0])
    goal = (18.0, 18.0)
    vel = [0.0, 0.0]
    traj = [state[:2].copy()]
    for _ in range(600):
        vel = controller.plan(state, goal, obstacles, vel)
        state = controller._motion(state, vel, controller.dt)
        traj.append(state[:2].copy())
        if np.hypot(state[0]-goal[0], state[1]-goal[1]) < 0.5:
            break
    traj = np.array(traj)
    for ox, oy in obstacles:
        ax.add_patch(plt.Circle((ox, oy), 0.2, color="gray", alpha=0.7))
    ax.plot(traj[:, 0], traj[:, 1], "r-", linewidth=2)
    ax.plot(1, 1, "go", markersize=10)
    ax.plot(18, 18, "bo", markersize=10)
    ax.set_title(f"DWA ({len(traj)} steps)")
    ax.set_xlim(0, 20); ax.set_ylim(0, 20); ax.set_aspect("equal")


def main():
    _, obstacles = make_obstacle_check()
    fig, axes = plt.subplots(2, 2, figsize=(12, 12))
    demo_astar(axes[0, 0])
    demo_rrt(axes[0, 1], obstacles)
    demo_informed(axes[1, 0], obstacles)
    demo_dwa(axes[1, 1])
    fig.suptitle("Path Planning Algorithms: A* / RRT* / Informed RRT* / DWA", fontsize=14)
    plt.tight_layout()
    out_dir = os.path.join(os.path.dirname(__file__), "..", "results")
    os.makedirs(out_dir, exist_ok=True)
    out = os.path.join(out_dir, "all_algorithms.png")
    plt.savefig(out, dpi=120)
    print(f"已保存: {out}")


if __name__ == "__main__":
    main()
