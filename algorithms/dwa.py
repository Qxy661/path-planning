"""
DWA 动态窗口法算法库（标准实现）

局部规划器。在速度空间采样 (v, w)，用轨迹预测评估，选最优速度执行。
特点：实时避障，计算量小，常用于 ROS/Nav2 的局部规划。

Usage:
    from dwa import DWA
    controller = DWA(obstacles)   # obstacles: [(x,y),...]
    vel = controller.plan(state, goal)  # 返回 (v, w)

原理：见 docs/03-局部规划(DWA).md
"""
import math
import numpy as np


class DWA:
    def __init__(self, max_speed=0.5, max_yaw_rate=1.0,
                 max_accel=0.5, max_dyaw_rate=1.0,
                 dt=0.1, predict_time=2.0, obstacle_radius=0.2):
        """初始化.
        max_speed: 最大线速度
        max_yaw_rate: 最大角速度
        max_accel: 最大线加速度
        """
        self.max_speed = max_speed
        self.max_yaw_rate = max_yaw_rate
        self.max_accel = max_accel
        self.max_dyaw_rate = max_dyaw_rate
        self.dt = dt
        self.predict_time = predict_time
        self.obstacle_radius = obstacle_radius
        # 目标/障碍/速度 权重
        self.w_goal = 0.5
        self.w_obstacle = 1.0
        self.w_heading = 0.3
        self.w_velocity = 0.2

    def _motion(self, state, vel, dt):
        """运动模型：更新位姿 (x, y, yaw)."""
        x, y, yaw = state
        v, w = vel
        x += v * math.cos(yaw) * dt
        y += v * math.sin(yaw) * dt
        yaw += w * dt
        return np.array([x, y, yaw])

    def _dynamic_window(self, state, vel):
        """速度动态窗口（考虑加速度限制）."""
        v, w = vel
        return [
            max(0.0, v - self.max_accel * self.dt),   # v_min
            min(self.max_speed, v + self.max_accel * self.dt),  # v_max
            max(-self.max_yaw_rate, w - self.max_dyaw_rate * self.dt),  # w_min
            min(self.max_yaw_rate, w + self.max_dyaw_rate * self.dt),  # w_max
        ]

    def _sample_velocities(self, v_range):
        """在动态窗口内采样速度对."""
        v_min, v_max, w_min, w_max = v_range
        velocities = []
        v_step = 0.05
        w_step = 0.1
        v = v_min
        while v <= v_max + 1e-6:
            w = w_min
            while w <= w_max + 1e-6:
                velocities.append([v, w])
                w += w_step
            v += v_step
        return velocities

    def _trajectory(self, state, vel):
        """预测轨迹（固定 predict_time 内）."""
        traj = [state]
        s = state
        for _ in range(int(self.predict_time / self.dt)):
            s = self._motion(s, vel, self.dt)
            traj.append(s)
        return np.array(traj)

    def _obstacle_dist(self, traj):
        """轨迹到最近障碍物的距离."""
        min_dist = float("inf")
        for point in traj:
            x, y = point[0], point[1]
            for obs in self.obstacles:
                dist = math.hypot(obs[0] - x, obs[1] - y)
                if dist < min_dist:
                    min_dist = dist
        return min_dist

    def _heading_cost(self, state, goal):
        """朝向代价：车头朝向与目标方向的夹角."""
        x, y, yaw = state
        goal_angle = math.atan2(goal[1] - y, goal[0] - x)
        diff = abs(yaw - goal_angle)
        while diff > math.pi:
            diff = 2 * math.pi - diff
        return diff

    def plan(self, state, goal, obstacles, vel=(0.0, 0.0)):
        """DWA 规划. 返回最优 (v, w) 速度.
        state: (x, y, yaw)
        goal: (x, y)
        obstacles: [(x,y),...]
        """
        self.obstacles = obstacles
        v_range = self._dynamic_window(state, vel)
        velocities = self._sample_velocities(v_range)

        best = None
        best_score = -float("inf")

        for vel_sample in velocities:
            traj = self._trajectory(state, vel_sample)
            # 障碍物代价
            obs_dist = self._obstacle_dist(traj)
            if obs_dist < self.obstacle_radius:  # 会碰撞，跳过
                continue

            # 三个目标代价
            goal_cost = abs(self._heading_cost(state, goal))
            obstacle_cost = 1.0 / obs_dist if obs_dist > 0 else 1.0 / 0.01
            heading_cost = self._heading_cost(state, goal)
            velocity_cost = abs(self.max_speed - vel_sample[0]) / self.max_speed

            # 加权总分（越小越好，取负）
            score = -(self.w_goal * goal_cost
                      + self.w_obstacle * obstacle_cost
                      + self.w_heading * heading_cost
                      + self.w_velocity * velocity_cost)

            if score > best_score:
                best_score = score
                best = vel_sample

        return best if best is not None else (0.0, 0.0)
