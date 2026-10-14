# 00 · 指南

AutoSim 文档入口：先分清**做什么 / 不做什么**，再按角色选后续章节。

| 相关 | 链接 |
|------|------|
| 能力边界 | [01 概述](01_overview.md) |
| 上手命令 | [03 快速开始](03_quickstart.md) |
| 包 README | [`../README.md`](../README.md) |

---

## 1. 一句话

AutoSim 把仿真世界（Habitat / Fake / Isaac）接到 **Autolink**：订阅 `/cmd_vel`，发布传感、TF、时钟与可选真值地图。  
**不实现** MapService / NavigationService / LocalizationService；SLAM 建图由 Cartographer 等外部节点完成。

---

## 2. 术语

| 术语 | 含义 |
|------|------|
| `habitat.*` | 配置根键（历史命名）；三种 backend 共用 |
| `WorldBackend` | 引擎无关协议：`pose` / `step` / `color_depth` / `laser_ranges` … |
| `nav` / `slam` | `habitat.mode`：是否发 GT `map→odom` 与 `/map` |
| 真值地图 | 仿真侧 OccupancyGrid / 点云，非 SLAM 结果 |
| `images/` | 演示动图，如 Isaac + Autoviz |

---

## 3. 仓库布局（与文档对应）

```text
autosim/
├── README.md           # 短入口 + 动图
├── docs/               # 本目录（00–10）
├── config/             # YAML 配置真相
├── autosim/            # Python 包（runner / backends / …）
├── scripts/run.sh      # 统一启动（Isaac 走 isaaclab python）
├── usd/                # Isaac 场景与机器人占位
├── urdf/               # 机器人 URDF
├── images/             # 演示素材
└── tests/              # pytest
```

---

## 4. 按角色导航

| 角色 | 路径 |
|------|------|
| 第一次跑通 | [03](03_quickstart.md) → Fake → Autoviz [08](08_map_and_viz.md) |
| 换后端 | [04](04_backends.md) → [05](05_configuration.md) |
| Isaac 室内场景 | [07](07_isaac.md) → [10](10_troubleshooting.md) |
| 改发布话题 | [06](06_topics.md) → `autosim/runner.py` |
| 加测试 | [09](09_testing.md) |
