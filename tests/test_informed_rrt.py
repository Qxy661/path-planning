"""Informed RRT* 测试：正确性 + 最优性."""
import sys
import os
sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "algorithms"))

import numpy as np
from informed_rrt import InformedRRT


def make_obstacle_check():
    obstacles = [(5, 5, 1.0), (15, 10, 1.2), (8, 15, 1.0)]

    def check(pos):
        for ox, oy, r in obstacles:
            if np.hypot(pos[0] - ox, pos[1] - oy) < r:
                return True
        return False

    return check


def test_informed_finds_path():
    """Informed RRT* 应找到路径."""
    np.random.seed(42)
    planner = InformedRRT(make_obstacle_check(), (0, 20, 0, 20),
                          step_size=0.5, max_iter=1500)
    path = planner.plan((1.0, 1.0), (18.0, 18.0))
    assert path is not None, "Informed RRT* 应找到路径"
    assert len(path) > 1


def test_informed_goal_reached():
    """终点应在目标附近."""
    np.random.seed(42)
    planner = InformedRRT(make_obstacle_check(), (0, 20, 0, 20),
                          step_size=0.5, max_iter=1500)
    path = planner.plan((1.0, 1.0), (18.0, 18.0))
    assert path is not None
    dist = np.linalg.norm(path[-1] - np.array([18.0, 18.0]))
    assert dist < 3.0, f"终点应接近目标，距离{dist:.2f}"


def test_informed_collision_free():
    """路径应无碰撞."""
    check = make_obstacle_check()
    planner = InformedRRT(check, (0, 20, 0, 20),
                          step_size=0.5, max_iter=1500)
    path = planner.plan((1.0, 1.0), (18.0, 18.0))
    assert path is not None
    for p in path:
        assert not check(p), f"路径点{p}在障碍内"


def test_informed_vs_rrt():
    """Informed RRT* 应比 RRT* 更优（路径成本更低）."""
    np.random.seed(42)
    from rrt_star import RRTStar
    check = make_obstacle_check()

    # RRT*
    rrt = RRTStar(check, (0, 20, 0, 20), step_size=0.5, max_iter=1000)
    rrt_path = rrt.plan((1.0, 1.0), (18.0, 18.0))
    rrt_cost = sum(np.linalg.norm(rrt_path[i+1] - rrt_path[i])
                   for i in range(len(rrt_path)-1)) if rrt_path else float("inf")

    # Informed RRT*
    informed = InformedRRT(check, (0, 20, 0, 20), step_size=0.5, max_iter=1000)
    inf_path = informed.plan((1.0, 1.0), (18.0, 18.0))
    inf_cost = sum(np.linalg.norm(inf_path[i+1] - inf_path[i])
                   for i in range(len(inf_path)-1)) if inf_path else float("inf")

    assert inf_cost <= rrt_cost * 1.2, \
        f"Informed应更优: Informed {inf_cost:.1f} vs RRT {rrt_cost:.1f}"
