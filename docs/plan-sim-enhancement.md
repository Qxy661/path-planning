# 仿真增强方案（已确认 · 2026-08-05）

> 调研驱动 + 可行性确认后的执行方案。两个方向：前沿探索自主建图（亮点优先）+ 局部规划器对比。

## 执行顺序（用户已确认）

1. **前沿探索自主建图**（先做，亮点优先）
2. **局部规划器对比**（后做，DWB/MPPI/RPP 三个）
3. 全局规划器统一用**手写 A\* 插件**（两亮点联动）

---

## 执行进度

- ✅ **Phase 1 前沿探索自主建图**（2026-08-05 完成，commit 9e568ce）
- 🔄 **Phase 2 局部规划器对比**（进行中）
- ⏳ Phase 3 规范化呈现 + 推送

---

## Phase 1 · 前沿探索自主建图

**目标**：机器人全自主探索未知迷宫，地图从空白自动生长到完整覆盖。

**架构**：
```
Gazebo 世界
  ↓
Cartographer（live SLAM，实时发布 /map）
  ↓
Nav2（不加载静态图，costmap 直接用 SLAM 的 /map）
  ↓
frontier_explore.py：订阅 /map → 前沿检测 → 聚类 → 贪心导航 → 循环
```

**核心算法**（`frontier_explore.py`）：
1. 订阅 `/map` → 提取栅格
2. **前沿单元** = 空闲格(0) 邻接 未知格(-1)
3. **连通域聚类** → 前沿区域
4. **目标选择** = 距当前位姿最近的前沿质心（贪心）
5. 经 Nav2 `NavigateToPose` 导航 → 到达后重扫 → 循环
6. 无前沿 = 建图完成

**指标**：覆盖率 vs 时间 / 探索总时长 / 地图完整度

**产出**：
- 自主建图视频（地图自动生长过程）
- 覆盖率增长曲线图
- `docs/10-前沿探索自主建图.md`

**技术要点/风险**：
- Nav2 需**去掉 map_server**（用 nav2_bringup navigation_launch.py 或自定义启动）
- Cartographer 的 /map + Nav2 costmap 的 topic 衔接
- QoS 坑（雷达/初始位姿）已知，直接复用之前的修复

---

## Phase 2 · 局部规划器对比

**目标**：同一场景量化对比 DWB / MPPI / Regulated Pure Pursuit。

**方法**（公平性关键）：
- 固定：同一地图、同一全局规划器（手写 A\*）、同一组起终点
- 唯一变量：切换 `controller_server` plugin
- 自动采集轨迹（`/amcl_pose`）

**指标**：到达时间 / 路径平滑度 / 避障成功率 / 轨迹长度 / 计算开销

**产出**：
- 量化对比表 + 三轨迹叠加图
- `docs/09-局部规划器对比.md`

---

## Phase 3 · 规范化呈现 + 推送

- 更新 README / results / 文档导航
- 全部量化实证保留
- git push

---

## 参考（调研确认的高质量项目）

- **前沿探索**：explore_lite / AniArka-Autonomous-Explorer / roadmap_explorer
- **局部规划器**：Nav2 官方 DWB/MPPI/RPP 配置 + 社区对比案例
