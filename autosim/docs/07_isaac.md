# 07 · Isaac 后端

Isaac Sim + USD：逼真 RGB-D、PhysX 碰撞、InteriorAgent 真值地图。演示：[`../images/isaac.gif`](../images/isaac.gif)。

| 相关 | 链接 |
|------|------|
| 快速启动 | [03](03_quickstart.md) |
| 地图可视化 | [08](08_map_and_viz.md) |
| 排错 | [10](10_troubleshooting.md) |
| USD 说明 | [`../usd/README.md`](../usd/README.md) |

---

## 1. 运行环境

| 要求 | 说明 |
|------|------|
| 解释器 | 必须用 Isaac Lab / Isaac Sim Python（`scripts/run.sh` 自动处理） |
| 默认路径 | `ISAACLAB_PATH=/workspace/isaaclab` |
| Autolink | 需 `cpython-311` 绑定（与 3.10 可并存） |
| Headless | Docker 默认；GUI：`AUTOSIM_ISAAC_HEADLESS=0` |

```bash
ISAACLAB_PATH=/workspace/isaaclab \
CONFIG=src/autonomy/autosim/config/isaac.yaml \
  src/autonomy/autosim/scripts/run.sh sim
```

---

## 2. 配置块 `habitat.isaac`

```yaml
isaac:
  headless: true
  physics:
    enabled: true
    block_motion: true      # 胶囊重叠则拒绝 set_pose
    freeze_scene: true      # 场景刚体改为 kinematic
    ensure_scene_colliders: true
    add_ground: false       # InteriorAgent 已有地板
    dt: 0.0166667
    robot_radius: 0.12
    robot_height: 0.28
```

| 场景类型 | `add_ground` | `ensure_scene_colliders` |
|----------|--------------|---------------------------|
| InteriorAgent | false | true（通常已有 CollisionAPI） |
| simple_room | 可 true | true（会给 Mesh 补 static collider） |

---

## 3. USD 与机器人

| 项 | 约定 |
|----|------|
| 世界 | `habitat.path` → 如 `usd/worlds/InteriorAgent/kujiale_0021/...` |
| 机器人 prim | `habitat.usd.robot_prim`（酷家乐常用 `/Root/Robot`） |
| 档案 | `robot.profile: turtlebot3_burger` / `husky` |
| 大场景 | `usd/worlds/InteriorAgent/` 被 `usd/.gitignore` 忽略，需本地下载 |

打开舞台后会：修复 faceVarying display\* primvar、挂相机、启用 PhysX、构建 floorplan 地图。

---

## 4. 相机与深度

- `sensors.camera.enabled: true` → `ENABLE_CAMERAS` + Replicator RGB / `distance_to_image_plane`。
- 深度清洗：`(W,H)` 转置为 `(H,W)`，远裁剪外 / 非有限值 → NaN；发布 `32FC1`。
- Autoviz 深度着色使用稳定米制色带（避免远点导致闪烁）。

冒烟：`scripts/smoke_isaac_camera.py`（经 `isaaclab.sh -p`）。

---

## 5. PhysX

| 步骤 | 行为 |
|------|------|
| 创建 | `isaacsim.core.api.world.World`（`stage_units_in_meters=1`） |
| 场景 | 非机器人刚体 → kinematic（防家具掉落） |
| 机器人 | 根 prim kinematic + 子 Capsule 碰撞体 |
| 步进 | `IsaacSimulator.step` → `world.step(render=…)` |
| 阻挡 | `block_motion` + overlap sphere；`reset`/`force` 可传送 |

源码：`autosim/backends/isaac/physics.py`。

---

## 6. 真值地图

| 话题 | 来源 |
|------|------|
| `/map` | InteriorAgent `rooms.json` 栅格化（自由 / 占据） |
| `/overall/map` | 优先 USD Mesh 顶点；否则密高墙线 + 地面填充 |

`map.enabled` / `map.publish` 在 `isaac.yaml` 默认 true。详见 [08](08_map_and_viz.md)。

---

## 7. 动态障碍与行人

与 Fake 共用 `Traffic`（`scenario.obstacle` / `pedestrian`）：

| 能力 | 行为 |
|------|------|
| 运动 | Runner 每拍 `traffic.step(dt)`；场地用 floorplan OccupancyGrid |
| `/scan` | 静态栅格射线 + `traffic.nearest_batch` 取近 |
| USD 障碍 / 行人 | 运动学胶囊（`obstacle_visual` / `pedestrian_visual`: `capsule` \| `invisible`） |
| 地图云 | `merge_traffic_into_map_cloud` 拼入 `/overall/map` |

碰撞与 `/scan` 由 CPU `Traffic` 负责；USD 只镜像位姿供 PhysX / RGB。

```yaml
habitat:
  isaac:
    traffic:
      usd_sync: true
      obstacle_visual: capsule
      pedestrian_visual: capsule
```

---

## 8. RL 运控模型（ONNX）

AutoSim 侧可选 `PolicyDriver`（`habitat.policy`），加载与 `policy_controller` 类似的 Twist ONNX，在无外部 `/cmd_vel`（或 `exclusive: true`）时写差速。

```yaml
habitat:
  policy:
    enabled: true
    model_path: /path/to/twist_policy.onnx
    rate_hz: 10.0
    exclusive: false   # true = 忽略 teleop
    use_laser: true
    laser_bins: 360
```

| 项 | 说明 |
|----|------|
| 依赖 | `pip install onnxruntime` |
| 输入 | 默认 `observation_kinematic` `[1,6]`；可选 `observation_laser` |
| 输出 | 默认 `action` → `(vx, wz)`，再按 `max_vx` / `max_wz` 裁剪 |
| 与 C++ 插件关系 | 完整观测栈 / Diffusion / TensorRT 仍用 `autonomy/control/.../policy_controller`；此处仅仿真闭环 |

栈内正式部署请用 ControllerServer + `policy_controller` 订 `/scan` 等话题（见该模块 README）。
