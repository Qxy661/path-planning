# 算法对比基准

> 实测数据（2026-08-04）。同一场景下 A* 与 RRT* 的性能对比。
> 附：手写算法 vs Nav2 系统对比。

## 对比结果

| 算法 | 路径 | 节点 | 耗时 | 最优性 |
|---|---|---|---|---|
| **A\*** | 27步 | 299 | 1.9ms | 保证最优 |
| **RRT\*** | 27步 | 179 | 150.3ms | 渐近最优 |

## 分析

- **A\***：图搜索，访问 299 节点，1.9ms——快且保证最优（适合已知静态图）
- **RRT\***：采样，179 节点，150ms——探索空间但慢（适合高维/复杂）

## 关键认知

- A* 快（启发式定向），RRT* 灵活（采样探索）
- 耗时差 79 倍：A* 1.9ms vs RRT* 150ms
- 路径长度相近（都是 27 步），但 A* 保证最优

## 手写算法 vs Nav2（仿真实测）

> ROS2 仿真（Gazebo + Nav2），详见 [07-ROS2仿真实战](../docs/07-ROS2仿真实战.md)。

| 维度 | 手写库（本项目） | Nav2 |
|---|---|---|
| 全局规划 | A* / RRT* / Informed RRT* | NavFn / Smac Planner |
| 局部规划 | DWA | DWB / Regulated Pure Pursuit |
| 定位 | ✗（假设已知位姿） | ✓ AMCL 概率定位 |
| 建图 | ✗（输入固定地图） | ✓ Cartographer SLAM |
| 接口 | 统一 `create_planner` | ROS2 Action（`NavigateToPose`） |
| 场景 | 20×20 静态栅格（Python） | 迷宫世界（Gazebo 仿真） |
| 产出 | 路径 + 可视化 | 定位 → 规划 → 避障 → 到达 |

### 量化对比（同一路线，真实地图 + 仿真实测）

> 路线 `(0.03,-0.84) → (1.5,1.5)`，起终点一致。
> 手写 A* 直接跑在 Cartographer 真实地图上（`exploration/astar_on_slam_map.py`）。

| 指标 | 手写 A* | Nav2 |
|---|---|---|
| **全局路径** | 3.95 m（平滑后 3.10 m）| 4.25 m（155 点规划）|
| **规划计算耗时** | 3.6 ms | —（实时规划器）|
| **访问节点** | 531 | — |
| **到达（含移动/定位/控制）** | — | 58.5 s ✅ |
| **成功率** | 1/1（地图规划）| 多次导航 100% 到达 |

**巡检（多目标自主导航）**：`(1.5,1.5) → (-1.5,-1.5) → (1.5,-1.5)` **3/3 到达**，总耗时 79.4s
![手写 A* 在真实地图上规划](astar_on_map.png)

**关键洞察**：
- **规划质量**：手写 A* 路径 3.10 m < Nav2 4.25 m（A* 图搜索全局最优；Nav2 兼顾运动学约束/安全距离）
- **计算效率**：A* 3.6 ms 完成规划，体现了启发式搜索的定向高效
- **系统完整性**：Nav2 多了定位（AMCL）、局部避障（DWB）、控制闭环——这是手写算法不具备的

### 进阶：手写 A* 直接作为 Nav2 全局规划器（插件集成）

> 手写 A* 通过 `nav2_core::GlobalPlanner` 插件化接入 Nav2（见 [08-手写算法集成Nav2插件](../docs/08-手写算法集成Nav2插件.md)）。

| 指标 | 结果 |
|---|---|
| 插件加载 | ✅ `Created global planner plugin GridBased of type astar_nav2_plugin/AstarPlanner` |
| 规划次数 | 158 次（A\* 持续作为 Nav2 全局规划器被调用）|
| 规划成功率 | **96.8%**（仅 5 次失败）|
| 实车导航 | 小车跨越迷宫，A\* 生成真实路径 |

**能力跃迁**：从"Python 里能跑算法"→"C++ 插件接入工业级导航框架"——这是作品集可信度最高的集成证明。

**意义**：手写算法证明**原理理解与最优性**，Nav2 证明**系统集成与落地**，二者互补构成完整导航能力链。

## 复现

```bash
python exploration/compare_benchmark.py       # 手写算法对比（A* vs RRT*）
python exploration/astar_on_slam_map.py       # 手写 A* 在真实地图规划
python robot/nav_metrics.py 1.5 1.5           # Nav2 导航指标采集
python robot/nav_goal.py --patrol "1.5,1.5;-1.5,-1.5;1.5,-1.5"   # 多目标巡检
```

## 复现

```bash
python exploration/compare_benchmark.py    # 手写算法对比
python robot/nav_goal.py 1.5 1.5           # Nav2 仿真导航
```
