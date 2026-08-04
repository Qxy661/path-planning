"""RRT* 算法测试：路径找到 + 渐近最优."""
import sys
import os
sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "algorithms"))

import numpy as np
from rrt_star import RRTStar


def make_obstacle_check():
    """标准障碍场景（3 个圆形障碍）."""
    obstacles = [(5, 5, 1.0), (15, 10, 1.2), (8, 15, 1.0)]

    def check(pos):
        for ox, oy, r in obstacles:
            if np.hypot(pos[0] - ox, pos[1] - oy) < r:
                return True
        return False

    return check


def test_rrt_finds_path():
    """RRT* 应找到路径（起点到终点）."""
    planner = RRTStar(make_obstacle_check(), (0, 20, 0, 20),
                      step_size=0.5, max_iter=1500)
    path = planner.plan((1.0, 1.0), (18.0, 18.0))
    assert path is not None, "RRT* 应找到路径"
    assert len(path) > 1, "路径应有多点"


def test_rrt_goal_reached():
    """路径终点应在目标附近."""
    planner = RRTStar(make_obstacle_check(), (0, 20, 0, 20),
                      step_size=0.5, max_iter=1500)
    path = planner.plan((1.0, 1.0), (18.0, 18.0))
    assert path is not None
    goal = path[-1]
    dist = np.linalg.norm(goal - np.array([18.0, 18.0]))
    assert dist < 3.0, f"终点应接近目标，距离{dist:.2f}"


def test_rrt_collision_free():
    """路径应避开障碍（无碰撞）."""
    check = make_obstacle_check()
    planner = RRTStar(check, (0, 20, 0, 20),
                      step_size=0.5, max_iter=1500)
    path = planner.plan((1.0, 1.0), (18.0, 18.0))
    assert path is not None
    # 采样检查路径点不在障碍内
    for p in path:
        assert not check(p), f"路径点{p}在障碍内"
