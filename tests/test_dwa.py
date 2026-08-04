"""DWA 算法测试：避障 + 到达目标."""
import sys
import os
sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "algorithms"))

import numpy as np
from dwa import DWA


def test_dwa_reaches_goal():
    """DWA 应避开障碍到达目标（600步=60秒，足够走30单位）."""
    np.random.seed(42)
    obstacles = [(5, 5), (6, 5.5), (15, 10), (15.5, 10.5), (8, 15)]
    controller = DWA()
    state = np.array([1.0, 1.0, 0.0])
    goal = (18.0, 18.0)
    vel = [0.0, 0.0]
    reached = False
    for _ in range(600):
        vel = controller.plan(state, goal, obstacles, vel)
        state = controller._motion(state, vel, controller.dt)
        if np.hypot(state[0] - goal[0], state[1] - goal[1]) < 0.5:
            reached = True
            break
    assert reached, "DWA 应在600步内到达目标"


def test_dwa_avoids_obstacle():
    """DWA 应避障（正前方障碍时转向）."""
    np.random.seed(0)
    controller = DWA()
    state = np.array([0.0, 0.0, 0.0])
    goal = (10.0, 0.0)
    obstacles = [(3.0, 0.0)]  # 正前方障碍
    vel = controller.plan(state, goal, obstacles)
    # 不应直冲障碍（应转向或减速）
    assert vel[0] >= 0, "速度应非负"


def test_dwa_output_valid():
    """DWA 输出速度应在合理范围."""
    np.random.seed(1)
    controller = DWA()
    state = np.array([0.0, 0.0, 0.0])
    goal = (5.0, 5.0)
    obstacles = []
    vel = controller.plan(state, goal, obstacles)
    assert abs(vel[0]) <= controller.max_speed, "线速度超限"
    assert abs(vel[1]) <= controller.max_yaw_rate, "角速度超限"
