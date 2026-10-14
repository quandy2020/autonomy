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
