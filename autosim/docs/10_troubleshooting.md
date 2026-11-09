# 10 · 排错

按症状定位；改代码前先确认解释器、配置路径与话题使能。

| 相关 | 链接 |
|------|------|
| 快速开始 | [03](03_quickstart.md) |
| Isaac | [07](07_isaac.md) |

---

## 1. 启动失败

| 症状 | 处理 |
|------|------|
| `No module named omni` / `isaacsim` | 必须用 `scripts/run.sh` 或 `isaaclab.sh -p`，不要用系统 `python3` 跑 Isaac |
| `autolink` 缺 311 `.so` | 用 Isaac Python 重编 pybind，拷到 `install/autonomy/python/autolink/` |
| `USD world not found` | 检查 `habitat.path`；InteriorAgent 需本地下载；可改 `simple_room.usd` |
| Config `ValueError` | 见 [05](05_configuration.md)；查重复 / 空通道 |

---

## 2. 有进程无图像

| 症状 | 处理 |
|------|------|
| Autoviz 无 RGB / depth | `camera.enabled: true`；Isaac 需 `ENABLE_CAMERAS`（`app/` 已设）；多等 warmup。`physics.enabled: false` 时仍会建 render-only World（否则 RTX 相机无帧）。外参走 URDF `camera_link`，勿只设负的 `sensor_height` 把镜头埋进机身 |
| 全黑 | 场景无灯（simple_room 已补光）；酷家乐确认 HDR / 材质 |
| Hydra primvar 刷屏 | 已有 `repair_display_primvars` + mute；升级后仍刷可再查 faceVarying |
| 深度像 RGB / 闪烁 | 确认深度为 `32FC1`；更新清洗与 Autoviz 稳定色带；勿把 RGB overlay 误当成深度 |

---

## 3. 地图与点云

| 症状 | 处理 |
|------|------|
| 无 `/map` | `mode: nav`、`map.enabled/publish: true`；Isaac 需旁路 `rooms.json`。`slam` 下 OccupancyGrid 由 Atlas 发，Autosim 不建 writer |
| 无 `/overall/map` | `map.enabled/publish: true` 且 `map.ply.channel` 非空；`slam` 下仍可发 |
| `/overall/map` 显示不全（Habitat） | 优先设 `map.ply.source` 为 `*_semantic.ply`；环视 ray 会被墙遮挡且受 `range_max` 截断 |
| `/overall/map` 只有墙线（Isaac） | 旧逻辑；应走 USD Mesh 采样（见 `stage.sample_mesh_cloud`） |
| `/overall/map` 整帧消失 | 点数 × `point_step` 可能超过 Autolink SHM 32 MiB → 增大 `stride` |
| Autoviz 点云稀疏 | 提高 PointCloud2 **Max Points**（≥ 发布点数）；配合 `stride` |
| `/scan` 与家具形状对不齐 | Isaac 需把 USD 家具刻进占用栅格（`mark_elevated_obstacles`）；Autoviz Fixed Frame=`map`；`nav` 下发 `map→odom` |
| 点云过大卡顿 | 增大 `map.ply.stride`；Autoviz Decimate |

---

## 3b. Chase / 机器人外观

| 症状 | 处理 |
|------|------|
| 无 `/camera/chase/image_raw` | `sensors.chase.enabled: true`；重启 sim 后才注册 Habitat 传感器 |
| chase 里看不到机身 | 需配置 `robot.urdf`；chase 会自动 instance；确认 PLY/URDF 路径可解析 |
| 机身朝向偏 90° | URDF 同步应为 `Ry(yaw)*Rx(−π/2)`，不要再用 agent 的 `yaw−π/2` |
| 机身太远 / 太近 | 调 `sensors.chase.distance_m` / `height_m`（TurtleBot 建议约 1.0 / 0.85） |
| 自车 RGB 被机身挡住 | chase 应只在 `render_chase` 时 show mesh；若长期可见查 `set_articulated_visible` |

---

## 4. 运动与碰撞

| 症状 | 处理 |
|------|------|
| 机器人穿墙 | `isaac.physics.enabled` + `block_motion`；确认胶囊半径；场景需有 collider |
| 完全动不了 | spawn 落在障碍内 → `set_pose(..., force=True)` 的 reset；或加大清空半径 / 改 spawn |
| 家具掉落 / 飞出 | `freeze_scene: true` |
| teleop 无响应 | 交互式 TTY 焦点；检查 `/cmd_vel` writer/reader |

---

## 5. 性能

| 症状 | 处理 |
|------|------|
| Isaac RGB ≪5Hz / 比 Habitat 慢很多 | 已在用 RTX Camera；勿在 50Hz PhysX 每拍再渲（仅 `color_depth` 采 RTX）。可降 `camera` 分辨率 |
| Isaac 帧率低 | headless；降相机分辨率 / `rate_hz`；关不需要的 lidar_3d |
| 仅要位姿 | `camera.enabled: false`（不加载相机 experience） |

---

## 6. 日志位置

- AutoSim：进程 stdout（Docker `docker exec` 终端）。
- Kit / PhysX：Isaac 日志通道；Hydra 已降到 error。
- Autoviz：`autoviz` 终端中的 SHM subscribe 行可确认通道发现。
