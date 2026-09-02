# Exploration stack

可插拔自主探索后端，由 `ExplorerFactory` 按配置选择实现。

## 目录结构

```
exploration/
  core/           # 接口、工厂、客户端、配置
  common/         # 地图环境、规划工具、可视化、点云/地形
  tare/           # TARE 分层探索（rgbd_tare / lidar_tare）
  far2d/          # FAR 2D 可见性图探索（far / rgbd_far）
  far3d/          # FAR 3D 轮廓 + 动态可见性图（far3d）
```

## 后端

| backend | 类 | 说明 |
|---------|-----|------|
| `rgbd_tare`（默认） | `TareExplorer` | TARE 分层：GridWorld TSP + viewpoint 覆盖 |
| `lidar_tare` | `TareExplorer`（LiDAR 模式） | 点云 + 地形图为主，无需深度相机 |
| `far` / `rgbd_far` | `FarExplorer` | 2D 可见性图 + NavFn 局部路径 |
| `far3d` | `far3d::Far3dExplorer` | 3D 轮廓检测 + 持久化 VG + 地形规划 |

## 主要话题

- `/exploration/waypoint` — 下发航点
- `/exploration/waypoint_reached` — 航点到达确认
- `/exploration/vg_markers` — 可见性图 MarkerArray
- `/exploration/reset` / `/exploration/pause` — 重置 / 暂停

## 配置与启动

- `config/exploration_autonomy.lua` — 默认 TARE
- `config/exploration_rgbd_far.lua` — FAR 2D
- `config/exploration_far3d.lua` — FAR 3D
- `autonomy/system/launch/exploration.launch`

## 编译

```bash
colcon build --packages-select autonomy
```
