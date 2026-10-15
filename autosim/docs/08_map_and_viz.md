# 08 · 地图与 Autoviz

真值地图用于导航联调与可视化；SLAM 模式下不发布 GT `/map`。

| 相关 | 链接 |
|------|------|
| 话题 | [06](06_topics.md) |
| Isaac 地图实现 | [07](07_isaac.md) |

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
      channel: /overall/map
      stride: 1
    grid:
      channel: /map
      resolution: 0.05
```

| 条件 | 结果 |
|------|------|
| `mode: slam` | 不发 GT 地图与 `map→odom` |
| `enabled: false` | 不构建 map_builder |
| `publish: false` | 可内部用栅格（如激光裁剪），但不建 writer |

---

## 2. 各后端地图来源

| 后端 | OccupancyGrid | 点云 |
|------|---------------|------|
| Habitat | navmesh 或环视投影 | 环视 ray / PLY `source` |
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

配置见 `isaac.yaml` 的 `map.robot_clearance_m` / `robot_height_m` / `mesh_inflate_m`（膨胀≈足迹半径）。`grid.z_min/z_max` 只影响 `/overall/map` 点云可视化范围，不直接当可通行带。

门：`rooms.json` 墙环会封住门洞，USD `door_*` 叶也会占格。默认 `free_doors: true`，按门 mesh 足迹把开口打成自由格（`door_opening_inflate_m`）。

---

## 3. Autoviz 步骤

1. 启动 AutoSim（`nav` + `map.publish`）。
2. 启动 Autoviz；**Fixed Frame** = `map`。
3. 添加显示：

| 显示 | 话题 |
|------|------|
| Image | `/camera/rgb/image_raw`、`/camera/depth/image_raw` |
| Map | `/map` |
| PointCloud2 | `/overall/map` |
| LaserScan | `/scan` |
| TF | `/tf` `/tf_static` |

`/overall/map` **采样一次**（密化只跑一遍），之后按 `rate_hz` **重发缓存**（Autolink 非 latch，晚订阅也能看到）。`/map` OccupancyGrid 同样按 `rate_hz` 刷新。

PointCloud2 建议：Color Transformer → `Intensity` 或 `AxisColor`；点数过大时用 **Decimate**，并配合 `map.ply.stride`。

演示：[`../images/isaac.gif`](../images/isaac.gif)。

---

## 4. 深度图显示

- 编码：`32FC1`（米）。
- Autoviz 用固定米制 Turbo 色带；无效 / NaN 为黑。
- 若曾闪烁：确认已更新 `image_utils` 与 Isaac 深度清洗（远裁剪、布局转置）。
