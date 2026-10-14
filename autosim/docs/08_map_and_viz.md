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
| Isaac | `rooms.json` | USD Mesh 顶点（优先） |

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

PointCloud2 建议：Color Transformer → `Intensity` 或 `AxisColor`；点数过大时用 **Decimate**，并配合 `map.ply.stride`。

演示：[`../images/isaac.gif`](../images/isaac.gif)。

---

## 4. 深度图显示

- 编码：`32FC1`（米）。
- Autoviz 用固定米制 Turbo 色带；无效 / NaN 为黑。
- 若曾闪烁：确认已更新 `image_utils` 与 Isaac 深度清洗（远裁剪、布局转置）。
