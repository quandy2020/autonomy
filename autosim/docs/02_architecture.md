# 02 · 架构

主循环与后端解耦：`Runner` 只依赖 `WorldBackend` 协议；引擎细节在 `backends/*`。

| 相关 | 链接 |
|------|------|
| 后端实现 | [04](04_backends.md) |
| 话题 | [06](06_topics.md) |
| 源码 | `autosim/runner.py` · `autosim/backend/` |

---

## 1. 分层

```text
┌─────────────────────────────────────────┐
│  teleop / 外部 /cmd_vel                 │
└──────────────────┬──────────────────────┘
                   ▼
┌─────────────────────────────────────────┐
│  Runner：时钟、差速、采样、编码、发布     │
│  Sensors · Map · Bridge · Messages      │
└──────────────────┬──────────────────────┘
                   ▼
┌─────────────────────────────────────────┐
│  WorldBackend（Protocol + registry）    │
├────────────┬────────────┬───────────────┤
│  habitat   │  fake      │  isaac        │
└────────────┴────────────┴───────────────┘
```

| 层 | 职责 |
|----|------|
| `Runner` | 周期：读 cmd_vel → 积分位姿 → `set_pose` / `step` → 采传感 → Autolink 发布 |
| `Robot` | 差速约束、odom、IMU 合成 |
| `Sensors` | 统一采样接口 + 可选噪声 |
| `Map` / floorplan | 真值栅格与点云缓存 |
| `Bridge` | 逻辑键 → 通道名 → protobuf writer |
| `WorldBackend` | `pose` / `step` / `color_depth` / `laser_ranges` / `world_cloud` … |

---

## 2. 一拍数据流（nav）

1. 订阅 `/cmd_vel`（TwistStamped）。
2. `Robot` 按 `control_hz` 积分，得到平面位姿。
3. `simulator.set_pose(x, y, yaw)`（Isaac 可因 PhysX 重叠拒绝更新）。
4. `simulator.step()`（Habitat 物理帧 / Fake no-op / Isaac `World.step`）。
5. 按各传感器 `rate_hz` 采样并 `Messages.encode_*`。
6. 发布 odom、TF、scan、相机、IMU、时钟；到期则发 `/map` 与点云。

---

## 3. 模式：`nav` vs `slam`

| | `nav`（默认） | `slam` |
|--|---------------|--------|
| `map→odom` | 发布（与 GT 对齐） | 不发（留给 Cartographer） |
| GT `/map` | `map.publish` 为 true 时发 | 强制不发 |
| 用途 | Autoviz / 规划联调 | 仿真里跑 SLAM |

逻辑在 `Runner.__init__`（`habitat.mode`）。

---

## 4. 注册与创建

```text
habitat.backend: isaac
        │
        ▼
backend.registry → backends/isaac.create(settings)
        │
        ▼
IsaacSimulator.create → App → USD → Sensors → Physics → FloorplanMap
```

新增后端：实现协议 → `register("name", factory)` → 增加 YAML 与文档 [04](04_backends.md)。
