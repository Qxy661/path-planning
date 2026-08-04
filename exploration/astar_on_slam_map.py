#!/usr/bin/env python3
"""
手写 A* 在真实 SLAM 地图上规划（与 Nav2 仿真对照）

把 Cartographer 构建的 room.pgm 栅格化 → 用本项目手写 A* 规划
→ 可视化路径 + 输出量化指标。起终点与 Nav2 仿真实测一致，
形成「纯算法 ↔ 真实建图 ↔ 系统导航」的闭环证据。

Usage:
    python exploration/astar_on_slam_map.py
"""
import os
import sys
import time

import numpy as np
from PIL import Image
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.font_manager import FontProperties

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "algorithms"))
from astar import AStar

# 中文字体
ZH = FontProperties(fname="/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc")

MAP_PGM = os.path.join(os.path.dirname(__file__), "..", "results", "room.pgm")
OUT_PNG = os.path.join(os.path.dirname(__file__), "..", "results", "astar_on_map.png")

# 与 Nav2 仿真实测一致的起终点（世界坐标, 米）
START_WORLD = (0.03, -0.84)
GOAL_WORLD = (1.5, 1.5)
RES = 0.05  # m/cell


def load_grid():
    """PGM → 栅格: 0=空闲(白), 1=障碍(占用/未知)."""
    arr = np.array(Image.open(MAP_PGM))
    return np.where(arr == 255, 0, 1)


def world_to_grid(wx, wy, ox, oy):
    """世界坐标 → 栅格坐标 (row, col)."""
    col = int((wx - ox) / RES)
    row = int((wy - oy) / RES)
    return row, col


def nearest_free(grid, r, c):
    """若点非空闲，找最近的空闲格."""
    if grid[r, c] == 0:
        return (r, c)
    R, C = grid.shape
    for d in range(1, 40):
        for dr in range(-d, d + 1):
            for dc in range(-d, d + 1):
                rr, cc = r + dr, c + dc
                if 0 <= rr < R and 0 <= cc < C and grid[rr, cc] == 0:
                    return (rr, cc)
    return (r, c)


def path_length_meters(path):
    """路径总长（米）."""
    if not path:
        return 0.0
    pts = np.array(path, dtype=float)
    segs = np.linalg.norm(np.diff(pts, axis=0), axis=1)
    return float(segs.sum() * RES)


def main():
    grid = load_grid()
    H, W = grid.shape
    print(f"地图栅格: {W}x{H} ({W*RES:.1f}x{H*RES:.1f} m)")

    # 读取 yaml origin
    import yaml
    meta = yaml.safe_load(open(os.path.join(os.path.dirname(MAP_PGM), "room.yaml")))
    ox, oy = meta["origin"][0], meta["origin"][1]

    start = nearest_free(grid, *world_to_grid(*START_WORLD, ox, oy))
    goal = nearest_free(grid, *world_to_grid(*GOAL_WORLD, ox, oy))
    print(f"起点(世界 {START_WORLD} → 栅格 {start})")
    print(f"终点(世界 {GOAL_WORLD} → 栅格 {goal})")

    # 手写 A*
    astar = AStar(grid, allow_diagonal=False)
    t0 = time.time()
    path, visited = astar.plan(start, goal)
    elapsed = (time.time() - t0) * 1000
    if path is None:
        print("❌ A* 无路径")
        return
    length_m = path_length_meters(path)
    print(f"A* 结果: 路径 {len(path)} 格 | {length_m:.2f} m | 访问 {visited} 节点 | 耗时 {elapsed:.1f} ms")

    # 平滑
    smooth = astar.smooth(path)
    smooth_m = path_length_meters(smooth)
    print(f"平滑后: {smooth_m:.2f} m（减少 {length_m-smooth_m:.2f} m）")

    # ---- 可视化 ----
    fig, ax = plt.subplots(figsize=(9, 8.8), dpi=130)
    # 底图: 白=空闲, 深蓝=占用, 浅灰=未知
    from matplotlib.colors import ListedColormap
    base = np.where(grid == 0, 1, 0)  # 用于显示
    img_arr = np.where(np.array(Image.open(MAP_PGM)) == 205, 2,
                       np.where(grid == 0, 1, 0))
    cmap = ListedColormap(["#2c3e50", "#ecf0f1", "#d5d8dc"])
    extent = [ox, ox + W * RES, oy, oy + H * RES]
    ax.imshow(img_arr, cmap=cmap, extent=extent, origin="upper", interpolation="nearest")

    # 路径
    path_arr = np.array(path)
    ax.plot(ox + (path_arr[:, 1] + 0.5) * RES, oy + (path_arr[:, 0] + 0.5) * RES,
            color="#e74c3c", linewidth=2.2, label=f"A* 路径 ({length_m:.2f} m)")
    if len(smooth) > 2:
        s_arr = np.array(smooth)
        ax.plot(ox + (s_arr[:, 1] + 0.5) * RES, oy + (s_arr[:, 0] + 0.5) * RES,
                color="#f39c12", linewidth=1.6, linestyle="--", label=f"平滑 ({smooth_m:.2f} m)")

    # 起终点
    ax.plot(ox + (start[1] + 0.5) * RES, oy + (start[0] + 0.5) * RES,
            "o", color="#27ae60", markersize=10, label="起点 (Nav2 实测起点)")
    ax.plot(ox + (goal[1] + 0.5) * RES, oy + (goal[0] + 0.5) * RES,
            "s", color="#8e44ad", markersize=10, label="终点 (Nav2 实测终点)")

    ax.set_xlabel("x (m)", fontsize=11)
    ax.set_ylabel("y (m)", fontsize=11)
    ax.set_title("手写 A* 在 Cartographer 真实地图上的规划", fontsize=13, pad=12, fontproperties=ZH)
    ax.legend(loc="lower left", fontsize=9, framealpha=0.9)
    ax.grid(True, color="#bdc3c7", linewidth=0.3, alpha=0.5)

    # 指标框
    info = (f"A*: {length_m:.2f} m · 访问 {visited} 节点 · {elapsed:.0f} ms\n"
            f"平滑后: {smooth_m:.2f} m")
    ax.text(0.02, 0.98, info, transform=ax.transAxes, fontsize=10,
            fontproperties=ZH, va="top", bbox=dict(boxstyle="round", fc="white", alpha=0.9))

    plt.tight_layout()
    plt.savefig(OUT_PNG, dpi=130, bbox_inches="tight")
    print(f"可视化已保存: {OUT_PNG}")

    # ---- 与 Nav2 对照摘要 ----
    print("\n=== 对照摘要（同起终点）===")
    print(f"手写 A*  : 路径 {length_m:.2f} m（平滑 {smooth_m:.2f} m）")
    print(f"Nav2     : 仿真实测自主到达（路径由 Nav2 规划器生成）")


if __name__ == "__main__":
    main()
