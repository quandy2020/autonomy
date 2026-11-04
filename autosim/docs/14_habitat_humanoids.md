# 14 · Habitat 动态人体（TrackVLA SMPL-X）

Habitat 后端可把 CPU `Traffic` 行人镜像为 **kinematic SMPL-X** 关节体，使 RGB-D 看到走动人体。平面运动仍走 agents YAML + SFM（与 Isaac 一致）；关节 walk/stop 动画来自 TrackVLA motion pickle。

相关：[04 后端](04_backends.md) · [Isaac 行人](07_isaac.md) · 本地 [`data/humanoids/`](../data/humanoids/)

---

## 1. 数据流

```text
scenario.pedestrian (agents YAML / SFM)
        │
        ▼
Runner → World.step → Traffic.poses
        │
        ▼
Simulator.step → HumanoidSync.sync
        │
        ├─ spawn URDF AO (MotionType.KINEMATIC)
        └─ WalkGaitController → joint_positions + gait offset
        │
        ▼
Habitat RGB-D / semantic   （激光仍用 Traffic.closer_actor）
```

**不做**：EVT `MultiHumanTrackingTask`、9-agent slot、OracleNav、panoptic detector、`habitat-lab` 依赖。

---

## 2. 配置

启用预设：

```bash
CONFIG=src/autonomy/autosim/config/habitat_people.yaml \
  src/autonomy/autosim/scripts/run.sh sim
```

或在任意 Habitat YAML 中：

```yaml
habitat:
  scenario:
    pedestrian:
      enabled: true
      social: true
      agents: config/scenarios/agents_habitat.yaml
  humanoids:
    enabled: true
    root: data/humanoids/humanoid_data
    visual: humanoid          # humanoid | none
    names: []                 # 空 = 目录轮询
    sync_hz: 30
    snap_navmesh: true
```

| 键 | 说明 |
|----|------|
| `humanoids.enabled` | 是否尝试创建 `HumanoidSync` |
| `humanoids.root` | TrackVLA 布局的 `humanoid_data` 目录 |
| `humanoids.visual` | `humanoid` 生成 AO；`none` 仅激光圆柱 |
| `humanoids.names` | 非空时只加载这些 avatar；空=目录全部 |
| `humanoids.skin_map` | `{1: male_46, …}`：agents `skin` → 文件夹名 |
| `AUTOSIM_HUMANOID_ROOT` | 覆盖 root（可为 `humanoids` 或 `humanoid_data`） |

### 单人外观（`agents_*.yaml`）

优先级：**`avatar` 名** → **`skin_map[skin]`** → **1-based `skin` 索引** → **按序号轮询**。

```yaml
agents:
  - id: 1
    pose: {x: -3.2, y: 0.5, yaw: 0.0}
    avatar: male_46          # 或 skin: "male_46"
    # skin: 1                # 1-based；可配合 humanoids.skin_map
    speed: 0.85
    behavior: regular
    goals: [{x: -2.0, y: 0.5}]
```

缺资产时只 warning，激光行人仍有效。外观变更（换 skin/avatar）会在下一 sync 重建对应 AO。

---

## 3. 资产

资产放在 **autosim 仓库内**（不依赖 TrackVLA 路径）：

```text
src/autonomy/autosim/data/humanoids/humanoid_data/<name>/<name>.urdf
src/autonomy/autosim/data/humanoids/humanoid_data/<name>/<name>_motion_data_smplx.pkl
```

下载（TrackVLA 同款 zip，写入上述目录）：

```bash
cd src/autonomy/autosim
python3 scripts/download_humanoids.py
# Drive 拦截时：浏览器下载 zip 后
# python3 scripts/download_humanoids.py --zip ~/Downloads/humanoids.zip
```

---

## 4. 源码

| 路径 | 作用 |
|------|------|
| `backends/habitat/humanoids/assets.py` | 发现 URDF + pkl |
| `backends/habitat/humanoids/controller.py` | walk/stop gait（精简自 TrackVLA） |
| `backends/habitat/humanoids/actor.py` | 单人 kinematic AO |
| `backends/habitat/humanoids/sync.py` | Traffic → AO |
| `backends/habitat/simulator.py` | `attach_humanoids` / `step` / `close` |

单测：`tests/test_habitat_humanoids.py`（无需 Kit）。
