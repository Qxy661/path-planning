# Path Planning · 路径规划与导航 🗺️

> 从算法认知 → 体系认知 → 应用认知，真做导航的应用能力 + 科研能力。
> 基于权威调研（LaValle《Planning Algorithms》、2024 综述、Nav2、MATLAB 科研角色）。

## 🎯 核心方法论

```
算法理解 → 手写实现 → 系统集成(Nav2) → 仿真验证 → 作品集
  (知识)    (能力)       (落地)        (demo)    (展示)
```

**三层认知**：
- **算法层**：A* / RRT* / DWA 原理与取舍
- **体系层**：算法在导航系统的位置（建图→定位→全局→局部→控制）
- **应用层**：MATLAB 仿真 vs ROS2 实际部署

## 📂 项目结构

```
path-planning/
├── algorithms/          # 手写算法库（Python）
│   ├── astar.py         # 图搜索类全局规划
│   ├── rrt_star.py      # 采样类全局规划
│   ├── dwa.py           # 局部规划（实时避障）
│   └── demo_all.py      # 三算法综合演示
├── matlab/              # MATLAB 科研验证对比
│   └── astar_matlab.m   # 手写 + Navigation Toolbox 对比
├── robot/               # TurtleBot3 巡检项目（经典+探索）
├── docs/                # 知识体系（算法→体系→应用）
└── results/             # 成果图
```

## 🚀 已完成的成果

### 手写算法库（Python）
| 算法 | 类 | 状态 |
|---|---|---|
| **A\*** | 图搜索全局规划 | ✅ 含平滑、启发式效率 |
| **RRT\*** | 采样全局规划 | ✅ rewiring 渐近最优 |
| **DWA** | 局部规划 | ✅ 实时避障 |

综合演示图：`results/algorithms_demo.png`

### MATLAB 对比（科研角色）
- `matlab/astar_matlab.m`：手写版 + Navigation Toolbox 版对比

## 🔄 进行中

- TurtleBot3 + Nav2 环境安装
- 室内巡检导航（经典应用）
- 探索应用（动态避障/手写替换 Nav2）
- 知识体系文档

## 📚 文档导航

| 文档 | 内容 |
|---|---|
| [作品集方案](references/m5-作品集方案.md) | 调研确认的完整方案 |
| [知识图谱](references/m5-路径规划知识图谱.md) | 三层认知框架 |
| docs/ | 知识体系（构建中）|

## License

MIT
