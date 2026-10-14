# 04 · 后端

三种 `habitat.backend` 实现同一 `WorldBackend` 契约；差异在场景表示与传感实现。

| 相关 | 链接 |
|------|------|
| 架构 | [02](02_architecture.md) |
| Isaac 专章 | [07](07_isaac.md) |

---

## 1. 对照

| | Habitat | Fake | Isaac |
|--|---------|------|-------|
| 配置 | `config/default.yaml` 等 | `config/fake.yaml` | `config/isaac.yaml` |
| 场景 | GLB / MP3D / scenario OBJ | scenario 体素 | USD（`usd/`） |
| 步进 | Habitat 物理 / 渲染 | 逻辑步进 | Kit + 可选 PhysX |
| RGB-D | Habitat 传感器 | 针孔 raycast | Replicator Camera |
| 激光 | `cast_ray` | 栅格 / 几何命中 | 标注器或占位 |
| 真值地图 | navmesh / 环视 PLY | FakeMap | `rooms.json` + USD 点云 |
| 包路径 | `backends/habitat` · `simulator.py` | `backends/fake` | `backends/isaac` |

---

## 2. Habitat

- 入口：`backends/habitat` 包装历史 `Simulator`。
- 适合数据集回放与 navmesh 占据。
- URDF 可挂载为 kinematic 关节体（视觉可隐藏以免遮挡相机）。

---

## 3. Fake

- 无外部引擎，CI 友好。
- `FakeWorld` + `FakeMap`：scenario 生成占据与点云；相机为 FoV 射线着色。
- 动态 `obstacle` / `pedestrian` 在控制周期更新，激光取更近命中。

---

## 4. Isaac

| 模块 | 文件 | 作用 |
|------|------|------|
| App | `app.py` | SimulationApp / AppLauncher 单例 |
| Stage | `usd_stage.py` | 打开 USD、机器人 Xform、网格点云采样 |
| Sensors | `sensors.py` | Camera RGB-D、深度清洗 |
| Physics | `physics.py` | PhysX World、胶囊、穿墙阻挡 |
| Floorplan | `floorplan_map.py` | InteriorAgent → `/map` |
| Simulator | `simulator.py` | 组装上述组件 |

细节、资源与 PhysX 参数见 [07](07_isaac.md)。

---

## 5. 扩展检查清单

1. 实现 `WorldBackend` 所需方法。  
2. `register("name", create)`。  
3. `config.py` 允许该 backend 字符串。  
4. 增加 `config/<name>.yaml`。  
5. 补 [09 测试](09_testing.md) 与本文表格一行。
