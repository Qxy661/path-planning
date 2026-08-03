"""
路径规划算法综合演示：A* / RRT* / DWA 在统一场景运行 + 可视化

用途：验证算法库可运行，生成作品集成果图。
对应：docs/02-全局规划(A*RRT).md + docs/03-局部规划(DWA).md
"""
import sys
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np

sys.path.insert(0, str(Path(__file__).parent))

from astar import AStar
from rrt_star import RRTStar
from dwa import DWA


def create_grid():
    """20x20 栅格地图（A* 用）."""
    grid = np.zeros((20, 20), dtype=int)
    grid[5:15, 7] = 1
    grid[10, 3:7] = 1
    grid[2:18, 13] = 1
    grid[15, 13:18] = 1
    return grid


def demo_astar(ax):
    """A* 演示."""
    grid = create_grid()
    start, goal = (1, 1), (18, 18)
    planner = AStar(grid)
    path, visited = planner.plan(start, goal)
    smooth_path = planner.smooth(path)

    ax.imshow(grid, cmap="Greys", origin="upper")
    if path:
        path_arr = np.array(path)
        ax.plot(path_arr[:, 1], path_arr[:, 0], "b-", linewidth=2, label="A* path")
        smooth_arr = np.array(smooth_path)
        ax.plot(smooth_arr[:, 1], smooth_arr[:, 0], "r-", linewidth=2, label="smoothed")
    ax.plot(start[1], start[0], "go", markersize=10, label="Start")
    ax.plot(goal[1], goal[0], "bo", markersize=10, label="Goal")
    ax.set_title(f"A* (visited {visited} nodes)")
    ax.legend(fontsize=8)


def make_obstacle_check():
    """障碍物检测函数（RRT* 用）：圆形障碍物."""
    obstacles = [(5, 5, 1.0), (15, 10, 1.2), (8, 15, 1.0), (12, 3, 0.8)]

    def check(pos):
        for ox, oy, r in obstacles:
            if np.hypot(pos[0] - ox, pos[1] - oy) < r:
                return True
        return False

    return check, obstacles


def demo_rrt(ax):
    """RRT* 演示."""
    check, obstacles = make_obstacle_check()
    start, goal = (1.0, 1.0), (18.0, 18.0)
    planner = RRTStar(check, (0, 20, 0, 20), step_size=0.5, max_iter=1000)
    path = planner.plan(start, goal)

    # 画障碍物
    for ox, oy, r in obstacles:
        circle = plt.Circle((ox, oy), r, color="gray", alpha=0.5)
        ax.add_patch(circle)
    ax.set_xlim(0, 20)
    ax.set_ylim(0, 20)
    ax.set_aspect("equal")

    if path:
        path_arr = np.array(path)
        ax.plot(path_arr[:, 0], path_arr[:, 1], "r-", linewidth=2, label="RRT* path")
    ax.plot(start[0], start[1], "go", markersize=10, label="Start")
    ax.plot(goal[0], goal[1], "bo", markersize=10, label="Goal")
    ax.set_title(f"RRT* ({len(planner.nodes)} nodes)")
    ax.legend(fontsize=8)


def demo_dwa(ax):
    """DWA 演示：从起点到目标，动态避障."""
    np.random.seed(42)
    obstacles = [(5, 5), (6, 5.5), (15, 10), (15.5, 10.5), (8, 15), (8.5, 14.5)]
    start = (1.0, 1.0, 0.0)  # x, y, yaw
    goal = (18.0, 18.0)
    controller = DWA()

    state = np.array(start)
    vel = [0.0, 0.0]
    traj = [state[:2].copy()]
    for _ in range(300):
        vel = controller.plan(state, goal, obstacles, vel)
        state = controller._motion(state, vel, controller.dt)
        traj.append(state[:2].copy())
        if np.hypot(state[0] - goal[0], state[1] - goal[1]) < 0.5:
            break

    traj = np.array(traj)
    for ox, oy in obstacles:
        circle = plt.Circle((ox, oy), 0.2, color="gray", alpha=0.7)
        ax.add_patch(circle)
    ax.set_xlim(0, 20)
    ax.set_ylim(0, 20)
    ax.set_aspect("equal")
    ax.plot(traj[:, 0], traj[:, 1], "r-", linewidth=2, label="DWA trajectory")
    ax.plot(start[0], start[1], "go", markersize=10, label="Start")
    ax.plot(goal[0], goal[1], "bo", markersize=10, label="Goal")
    ax.set_title(f"DWA (reached in {len(traj)} steps)")
    ax.legend(fontsize=8)


def main():
    fig, axes = plt.subplots(1, 3, figsize=(18, 6))
    demo_astar(axes[0])
    demo_rrt(axes[1])
    demo_dwa(axes[2])
    fig.suptitle("Path Planning Algorithms: A* / RRT* / DWA", fontsize=14)
    plt.tight_layout()
    out_dir = Path(__file__).parent.parent / "results"
    out_dir.mkdir(exist_ok=True)
    out = out_dir / "algorithms_demo.png"
    plt.savefig(out, dpi=130)
    print(f"综合演示已保存: {out}")


if __name__ == "__main__":
    main()
