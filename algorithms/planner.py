"""
统一 Planner 接口

所有算法统一调用方式（专业/可复用）：
    planner = create_planner("astar", ...)
    path = planner.plan(start, goal)

支持：astar / rrt_star / informed_rrt
"""
import numpy as np

from astar import AStar
from rrt_star import RRTStar
from informed_rrt import InformedRRT


def create_planner(name, **kwargs):
    """创建规划器（统一接口）.
    name: "astar" / "rrt_star" / "informed_rrt"
    kwargs: 算法参数
    """
    name = name.lower().replace("-", "_").replace(" ", "_")

    if name in ("astar", "a_star", "a*"):
        return AStar(**kwargs)
    elif name in ("rrt_star", "rrt*", "rrtstar"):
        return RRTStar(**kwargs)
    elif name in ("informed_rrt", "informed_rrt*"):
        return InformedRRT(**kwargs)
    else:
        raise ValueError(f"未知算法: {name}. 支持: astar/rrt_star/informed_rrt")


def plan(planner, start, goal):
    """统一规划入口（自动适配不同算法）."""
    if isinstance(planner, AStar):
        path, visited = planner.plan(start, goal)
        return {
            "path": path,
            "nodes": visited,
            "type": "astar",
        }
    elif isinstance(planner, (RRTStar, InformedRRT)):
        path = planner.plan(start, goal)
        return {
            "path": path,
            "nodes": len(planner.nodes),
            "type": "rrt" if isinstance(planner, RRTStar) else "informed_rrt",
        }
    else:
        raise TypeError(f"不支持的规划器: {type(planner)}")


# 演示用法
if __name__ == "__main__":
    # A*（栅格）
    grid = np.zeros((20, 20), dtype=int)
    grid[5:15, 7] = 1
    astar = create_planner("astar", grid=grid)
    r = plan(astar, (1, 1), (18, 18))
    print(f"A*: {len(r['path'])}步, {r['nodes']}节点")

    # RRT*（连续）
    def check(pos):
        for ox, oy, rr in [(5, 5, 1.0), (15, 10, 1.2)]:
            if np.hypot(pos[0]-ox, pos[1]-oy) < rr:
                return True
        return False
    rrt = create_planner("rrt_star", obstacle_check=check, bounds=(0, 20, 0, 20))
    r = plan(rrt, (1, 1), (18, 18))
    print(f"RRT*: {len(r['path'])}步, {r['nodes']}节点")
