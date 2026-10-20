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
    mode: wheeled           # 差速 USD（turtlebot / husky）
    block_motion: true
    freeze_scene: true      # 场景刚体改为 kinematic
    ensure_scene_colliders: true
    add_ground: false       # InteriorAgent 已有地板
    dt: 0.0166667
    robot_radius: 0.12
    robot_height: 0.28
```

| `mode` | 行为 |
|--------|------|
| `wheeled` | 差速 USD + 轮关节；碰撞来自机器人 USD；无轮关节则报错 |

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
- 可选 `camera.surround: [{name, yaw_deg, xyz?, hfov_deg?}, …]` 挂多目。

冒烟：`scripts/smoke_isaac_camera.py`（经 `isaaclab.sh -p`）。

### 4.1 `/scan` 后端

Isaac 模式默认 **只用 Kit RTX 传感器**（相机 / 2D Lidar / IMU），不再静默回退 CPU 射线。

| `sensors.lidar_2d.backend` | 行为 |
|----------------------------|------|
| `rtx`（Isaac 默认） | Isaac RTX Lidar + FlatScan（`rtx_config` 须为 2D，默认 `RPLIDAR_S2E`） |
| `cpu` | floorplan 射线 + Traffic 圆柱（调试） |
| `merge` | 取更近命中（调试） |

> `Example_Rotary` 是 3D（elevationDeg≠0），挂 FlatScan 会报错并得到空 `/scan`。

### 4.2 关节 / 轮式 / 随机 / 调试

| 项 | 配置 |
|----|------|
| `/joint_states` | `sensors.joint_states.enabled`（轮关节有数据时） |
| 差速 PhysX | `isaac.physics.mode: wheeled` + `isaac.wheeled.*` |
| 域随机 | `isaac.domain_random.enabled`（光照 / 行人外观） |
| 种子 | `isaac.seed` |
| 暂停 / 单步 | `IsaacWorld.pause` / `resume` / `step_once` |
| 录屏 | `isaac.record_path`（best-effort viewport） |

---

## 5. PhysX

| 步骤 | 行为 |
|------|------|
| 创建 | `isaacsim.core.api.world.World`（`stage_units_in_meters=1`） |
| 场景 | 非机器人刚体 → kinematic（防家具掉落） |
| 机器人（wheeled） | 机器人 USD 碰撞 + 轮关节驱动 |
| 步进 | `IsaacWorld.step` → `world.step` |
| 阻挡 | 可选 `block_motion` + overlap |

源码：`autosim/backends/isaac/physics.py`、`robots/wheeled.py`。

---

## 6. 真值地图

| 话题 | 来源 |
|------|------|
| `/map` | `scene/floorplan.py` 栅格化 InteriorAgent `rooms.json` |
| `/overall/map` | 优先 USD Mesh 顶点；否则密高墙线 + 地面填充 |

`map.enabled` / `map.publish` 在 `isaac.yaml` 默认 true。详见 [08](08_map_and_viz.md)。

---

## 7. 动态障碍与行人

无 ROS。CPU 侧 `World` 管理场地、占据与社会力运动；Isaac 只镜像 USD **真人角色**（或可选胶囊）供 RGB。

HuNavSim 兼容层（Autolink `/compute_agents`、`automsgs/hunav_msgs`、场景 YAML）见 **[12_hunav_isaac.md](12_hunav_isaac.md)**。

| 能力 | 行为 |
|------|------|
| 环境 | `scene/world.py`：从 floorplan 填尺寸，绑定 `Field`，每拍 `world.step(dt, robot)` |
| 社会力 | `scene/agents/force.py`：目标吸引 + 行人互斥 + 障碍排斥 |
| 反应 | `regular` / `impassive` / `surprised` / `scared` / `curious` / `threatening` |
| 配置 | `pedestrian.agents` → `config/scenarios/agents_isaac.yaml` |
| `/scan` | 静态栅格射线 + `traffic.nearest_batch` 取近（仍走 CPU 圆柱） |
| USD | `pedestrian_visual: character` → 引用 Isaac People 人体 USD，每拍同步位姿 |
| 地图云 | 动态体表面点拼入 `/overall/map` |

```yaml
habitat:
  isaac:
    traffic:
      usd_sync: true
      obstacle_visual: none
      pedestrian_visual: character   # character | capsule | none
      # character_root: usd/people/Characters
  scenario:
    obstacle:
      enabled: false
    pedestrian:
      enabled: true
      social: true
      agents: config/scenarios/agents_isaac.yaml
```

**角色资产**：把 Isaac Sim 的 `People/Characters` 拷到 `autosim/usd/people/Characters/`，或设 `character_root` / `AUTOSIM_PEOPLE_ROOT`。未找到 USD 时只告警，激光行人仍有效。

| 配置 | 含义 |
|------|------|
| `character_yaw_offset_deg` | People 朝向修正（默认 -90） |
| `character_z` | 脚底高度 |
| `collision_proxy` | 隐形运动学胶囊，供 PhysX 碰人 |
| `sync_hz` | USD 位姿更新频率（逻辑步进仍全频） |
| `pedestrian_anim` | UsdSkel walk/idle（共享时间轴相位） |
| `anim_walk_mps` | 平面速度对应该 clip 1× 播放 |
| `animation_root` | 可选 Animations 目录（含 walk/idle USD） |
| `scenario.pedestrian.laser_radius` | `/scan` 与地图云足迹（可大于 `radius`） |

出生点 / 航点若落在占据格上，绑定 floorplan 后会自动 `relocate` 到自由空间。

地面导航联调请用 [`config/isaac_nav.yaml`](../config/isaac_nav.yaml)（turtlebot + wheeled PhysX，关 ONNX）。

录制：`habitat.dataset.include_people: true` 时写入 `observation.people` 与 `episode_*_people.jsonl`。

---

## 8. RL 运控模型（ONNX）

`PolicyDriver`（`habitat.policy`）支持两种 `action_space`：

### 8.1 平面 Twist（默认）

与 `policy_controller` 类似的 Twist ONNX；无外部 `/cmd_vel`（或 `exclusive`）时写差速。

```yaml
habitat:
  policy:
    enabled: true
    action_space: twist
    model_path: /path/to/twist_policy.onnx
    rate_hz: 10.0
    exclusive: false
    use_laser: true
    laser_bins: 360
```

| 项 | 说明 |
|----|------|
| 输入 | `observation_kinematic` `[1,6]`；可选 laser |
| 输出 | `(vx, wz)`，按 `max_vx` / `max_wz` 裁剪 |

栈内正式 Twist 部署请用 ControllerServer + `policy_controller`。
