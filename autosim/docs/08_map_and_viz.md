# 08 · 地图与 Autoviz

真值地图用于导航联调与可视化。`mode: slam` 不发布 GT OccupancyGrid `/map`
与 `map→odom`（留给 Atlas / Cartographer）；`map.publish: true` 时仍可发
GT 点云 `/overall/map`。

| 相关 | 链接 |
|------|------|
| 话题 | [06](06_topics.md) |
| Isaac 地图实现 | [07](07_isaac.md) |
| Atlas 验证 | [13](13_atlas_validation.md) |

---

## 1. 开关

```yaml
habitat:
  mode: nav
  map:
    enabled: true
    publish: true
    frame: map
    rate_hz: 1.0
    ply:
      # 可选：外部语义 PLY（大场景优先）；否则 Habitat 环视 ray-cast
      # source: /path/to/scene_semantic.ply
      channel: /overall/map
      stride: 1
    grid:
      channel: /map
      resolution: 0.05
      z_min: -1.0
      z_max: 1.5
```

| 条件 | 结果 |
|------|------|
| `mode: slam` | 不发 OccupancyGrid `/map` 与 `map→odom`；`/overall/map` 仍受 `publish` 控制 |
| `enabled: false` | 不构建 map_builder |
| `publish: false` | 可内部用栅格（如激光裁剪），但不建 writer |
| `ply.source` 存在 | 从 PLY 加载点云（经 `grid.z_*` / `ply.z_*` 过滤）；否则环视 ray-cast |

大 PLY（百万级顶点）务必设 `stride ≥ 5`，否则可能超过 Autolink SHM **32 MiB**
硬上限导致整帧丢失。Autoviz PointCloud2 **Max Points** 也需 ≥ 发布点数。

---

## 2. 各后端地图来源

| 后端 | OccupancyGrid | 点云 |
|------|---------------|------|
| Habitat | navmesh 或环视投影 | **优先** `ply.source`；否则环视 ray（单点遮挡 + `range_max` 会截断大房子） |
| Fake | scenario 栅格 | scenario 点 |
| Isaac | `rooms.json` + 机身高度带内的 USD 障碍 | USD Mesh **面片加密**采样（墙体实心） |

Isaac 默认用 `rooms.json` 挤出的密高墙点云（实心、启动快）。`map.ply.surface_spacing_m > 0` 才对 USD 面片 densify（慢）。点云在主线程编码一次并缓存重发——勿走 SensorWorker（会被相机挤掉）。

### Isaac 二维占用高度带（可通行）

地面移动机器人只关心**机身高度切片**内的障碍，不是整间屋子的高度：

| 高度 | 含义 | 栅格 |
|------|------|------|
| `z < robot_clearance_m` | 地面 / 地毯噪声 | 保持自由 |
| `clearance ≤ z ≤ robot_height_m + margin` | 与机身相交（沙发、椅腿、墙根） | 占用 |
| `z > body top` | 悬空桌面、吊灯 | 忽略（可钻过） |

配置见 `isaac.yaml` 的 `map.robot_clearance_m` / `robot_height_m` / `mesh_inflate_m`（膨胀≈足迹半径）。`grid.z_min/z_max` 管 `/overall/map` 墙柱高度带；**地面点固定 z=0**（与 InteriorAgent 地板 / `base_link_height≈0.01` 对齐）。勿把 `z_min` 设成 0.1，否则点云会悬空。

门：`rooms.json` 墙环会封住门洞，USD `door_*` 叶也会占格。默认 `free_doors: true`，按门 mesh 足迹把开口打成自由格（`door_opening_inflate_m`）。

---

## 3. Autoviz 步骤

1. 启动 AutoSim（`map.publish: true`；Atlas 用 `atlas.yaml`）。
2. 启动 Autoviz（Atlas 建议 `--config .../atlas.autoviz`）；**Fixed Frame** = `map`。
3. 添加显示：

| 显示 | 话题 |
|------|------|
| Image | `/camera/rgb/image_raw`、`/camera/depth/image_raw` |
| Image（俯视） | `/camera/topdown/image_raw`（`sensors.topdown`：默认正交 + `fit_scene` 罩住整层） |
| Image（第三人称） | `/camera/chase/image_raw`（`sensors.chase`：身后跟随，看机器人运动） |
| Map | `/map`（全局占用栅格俯视；`slam` 下通常由 Atlas 发） |
| PointCloud2 | `/overall/map`（全局场景点云俯视） |
| LaserScan | `/scan` |
| RobotModel | `/robot_description`（autosim 发布 URDF **文件绝对路径**，Autoviz 按目录解析 `meshes/`） |
| TF | `/tf` `/tf_static`（`chase_link` 在 `/tf` 上随底盘动态发布；`topdown_link` 在 `/tf_static`） |

全局俯视优先用 **Map `/map`** + **PointCloud2 `/overall/map`**（Fixed Frame=`map`）。
需要看整层着色时再开 topdown；需要看机身相对场景运动时开 chase。

`/overall/map` **采样一次**（密化只跑一遍），之后按 `rate_hz` **重发缓存**（Autolink 非 latch，晚订阅也能看到）。`/map` OccupancyGrid 同样按 `rate_hz` 刷新。

PointCloud2 建议：Color Transformer → `RGB8` / `Intensity` / `AxisColor`；点数过大时用 **Decimate**，并配合 `map.ply.stride`。

演示：[`../images/isaac.gif`](../images/isaac.gif)。

---

## 4. Habitat 第三人称 chase

```yaml
sensors:
  chase:
    enabled: true
    distance_m: 1.0      # 身后距离（TurtleBot 建议 ~1 m）
    height_m: 0.85
    look_at_height_m: 0.15
    hfov_deg: 70.0
    rate_hz: 10.0
    channel: /camera/chase/image_raw
```

- 相机挂在 agent 身后上方，随机器人平移/转向。`chase_link` 是 `/tf` 上 `base_link` 的动态子系，不要放进 `/tf_static`。
- `enabled: true` 时自动 `add_articulated_object_from_urdf`；**仅** `render_chase` 时显示 mesh，自车 RGB-D 不被挡。
- URDF 朝向：`Ry(yaw) * Rx(−π/2)`（ROS z-up → Habitat y-up）；勿再套 agent 的 `yaw−π/2`。
- 实现：`autosim/bridge/chase.py`、`Simulator.render_chase`、`Runner.submit_chase`。

---

## 5. 深度图显示

- 编码：`32FC1`（米）。
- Autoviz 用固定米制 Turbo 色带；无效 / NaN 为黑。
- 若曾闪烁：确认已更新 `image_utils` 与 Isaac 深度清洗（远裁剪、布局转置）。
