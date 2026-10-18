# 地图与建图

## 1. 在线几何

| 组件 | 路径 | 职责 |
|------|------|------|
| `VoxelMapManager` | `map/voxel_map.*` | 点–面残差、体素平面、地图更新 |
| `IkdTree` | `map/ikd_tree.*` | 增量 kNN / 半径查询、导出采样 |
| `VisualMap` / `VisualPoint` | `map/visual_*.` | 稀疏视觉地标 |
| `MapSession` | `map/map_session.*` | 会话锚定、轨迹、跨会话 id |
| `OccupancyGridMap` / `Builder` | `map/occupancy_grid_*.` | 2.5D hit/visit 占用栅格（atlas g2p5） |

`LivoOdometry` 在 LIO 成功更新后写体素图；可选同步关键帧到 `MapBuilder`。

与重定位用的 `PriorOccupancyMap`（3D 稀疏哈希）不同：本栅格面向导航/显示，分层子网格 + 射线 miss 刷白。

## 2. MapBuilder

路径：`mapping/map_builder.*`。

```cpp
MapBuilder builder(cfg);
builder.AddKeyFrame(kf);
builder.AddVisualLandmark(pos);
builder.Save(dir);                    // 地图包 v3
auto prior = builder.ExportPriorMap(); // → RelocalizationManager
```

`LivoOdometry`：`mapping.enable` 时 `TrySubmitKeyFrame` → builder；`SaveBuiltMap` / `LoadBuiltMap`。

可选 2.5D 占用栅格（默认关）：

```cpp
cfg.build_occupancy_grid = true;
cfg.occupancy_grid.map.resolution = 0.1f;
// AddKeyFrame → OccupancyGridBuilder::AddScan
// Save → occupancy.png（OpenCV）
```

## 3. 地图包格式（v3）

原则：目录包默认持久化；写盘原子化（`*.tmp` + rename）；可选指纹校验。

`MapBuilder::Save` 写出：

| 文件 | 内容 |
|------|------|
| `map_meta.txt` | `argus_map_package <ver>` + 分辨率 / 点数 / `map_xyz_fnv64` |
| `session.txt` | `MapSession`：锚定 + 关键帧轨迹 |
| `map.xyz` | 世界系 LiDAR 点 |
| `prior.xyz` | 重定位占据先验（`PriorMapLoader` 优先） |
| `visual.xyz` | 视觉地标 `x y z nx ny nz` |
| `trajectory.tum` | TUM 轨迹 |
| `keyframes/index.txt` + `*.bin` | 可选 body 点云（跨会话 SC） |
| `occupancy.png` | 可选 2.5D 占用栅格（`build_occupancy_grid`） |

加载：`MapBuilder::Load`（指纹）/ `LivoOdometry::LoadBuiltMap`（`SetPriorMap` + 有 keyframes 时注册 SC）。  
`PriorMapLoader::LoadPath`：`.xyz` 或包目录（`prior.xyz` → `map.xyz`）。  
`VerifyMapPackage`：指纹校验。

兼容：v1/v2 无指纹时 `map_xyz_fnv64=0` 跳过校验；布局破坏性变更须 bump `kMapPackageVersion`。

## 4. 自清洁与置信度

路径：`map/map_confidence.*`、`map/map_self_cleaning.*`。

```cpp
BumpMapConfidence(&voxel->confidence_, under_degeneracy, cfg);
MapSelfCleaning cleaner(cfg);
cleaner.Start();
cleaner.UpdateConfidence(map, degeneracy);   // shared_lock
cleaner.MaybeRequestPrune(&map, now_sec);    // 非阻塞
```

| 规则 | 行为 |
|------|------|
| 高质量观测 | `confidence ↑`，`observation_count++` |
| 退化下更新 | 乘 `degeneracy_penalty` 并扣分 |
| Prune | `confidence < τ` 且 `observation_count < N_min` → 删根体素 |
| 高观测锚点 | 即使置信度暂时偏低也保留 |

配置：`config/robustness/map_cleaning.yaml`；默认 `enable: false`。

锁约定见 [03_sequence.md §6](03_sequence.md)。禁止 ESIKF 迭代内 `Prune`。

自清洁管单会话弱观测；跨会话环境变化见 [07_loop_closure.md](07_loop_closure.md)。

## 5. 测试

- `test_ikd_tree`：Build / Add / kNN / Radius
- `test_map_builder`：Save/Load + ExportPriorMap
- `test_map_cleaning`：弱体素删除、高观测锚点保留
- `test_occupancy_grid`：Init/Resize/Occupied/ToCv + Builder 走廊

## 6. 2.5D 占用栅格

路径：`occupancy_grid_data.hpp`、`occupancy_grid_subgrid.*`、`occupancy_grid_map.*`、`occupancy_grid_builder.*`。

| API | 行为 |
|-----|------|
| `OccupancyGridMap::Init/Resize` | 按世界 XY 分配分层子网格 |
| `SetHitPoint` / `SetMissPoint` | 障碍命中 / 射线自由空间（高度门控） |
| `Occupied` / `ToCv` | 占用判定与灰阶渲染 |
| `OccupancyGridBuilder::AddScan` | 位姿 + 点云 → 扩边 + 360° 投影 |

地板高度带：`default_floor_height` + `[min_th_floor, max_th_floor]`（相对地面）。
