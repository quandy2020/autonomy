# 12 · HuNav ↔ Isaac（Autolink）

把 [Hunav_isaac_wrapper](https://github.com/robotics-upo/Hunav_isaac_wrapper) 的组织迁到 **autosim + automsgs**，用 **Autolink** 替代 ROS 2，对齐 HuNavSim 的 `/compute_agents` 契约。

| 相关 | 链接 |
|------|------|
| Isaac 行人 / People | [07](07_isaac.md) §7 |
| 消息定义 | `automsgs/proto/msgs/hunav_msgs/` |
| 场景 YAML | `config/scenarios/hunav/` |

---

## 1. 关系

```text
┌─────────────────────────────┐     Autolink      ┌──────────────────────────────┐
│ Isaac Sim (USD / People)    │  /compute_agents  │ autosim Runner               │
│ backends/isaac/hunav/       │ ────────────────► │ scene/hunav/ComputeAgents    │
│  HuNavManager (wrapper)     │ ◄──────────────── │  Pedestrian + Force (SFM)    │
│  WorldBuilder / AnimGraph   │   updated Agents  │ HunavServiceHost             │
└─────────────────────────────┘                   └──────────────────────────────┘
         ▲                                                    │
         │ pack Agents + robot                                │ automsgs.hunav_msgs
         └────────────────────────────────────────────────────┘
```

| 角色 | 职责 |
|------|------|
| **Isaac** | 按 [Hunav_isaac_wrapper](https://github.com/robotics-upo/Hunav_isaac_wrapper) 生成 SkelRoot + AnimationGraph；PhysX 射线填 `closest_obs`；打包 Agents |
| **autosim** | 社会力 / 反应行为；提供 `/compute_agents`（也可进程内 `ComputeAgentsEngine`） |
| **automsgs** | `Agent` / `Agents` / `AgentBehavior` + `ComputeAgents` 等 srv，对齐 `hunav_msgs` |

进程内默认：Runner 仍用 CPU `Traffic` 步进行人，Isaac `PeopleSync` 镜像 USD（见 [07](07_isaac.md) §7）。  
打开 `habitat.hunav.service: true` 后，外部包装器可通过 Autolink 调同一套 SFM。  
打开 `habitat.hunav.manager: true` 后，走 **wrapper 路径**：禁用 PeopleSync，在 `{World\|Root}/Characters` 生成角色 + AnimGraph；`own_motion: true` 时跳过 `Traffic.pedestrian.step`，由 `step_hunav` 驱动。

消息字段对齐 HuNav：`id` / `name` / `group_id` / `skin` / `radius` / `goal_radius` / 行为力增益 / goals / `closest_obs`。`skin` 映射与上游一致（0=随机，1..11=指定模型）。

---

## 2. 消息（automsgs）

| Proto | HuNav 对应 |
|-------|------------|
| `msgs/hunav_msgs/AgentBehavior` | `AgentBehavior.msg` |
| `msgs/hunav_msgs/Agent` | `Agent.msg` |
| `msgs/hunav_msgs/Agents` | `Agents.msg` |
| `srvs/ComputeAgents` | `/compute_agents` |
| `srvs/ComputeAgent` | `/compute_agent` |
| `srvs/MoveAgent` | `/move_agent` |

枚举取值与 HuNav 一致（`PERSON=1`、`BEH_REGULAR=1` …）。生成后 Python 位于 `automsgs.msgs.hunav_msgs.*_pb2`。

---

## 3. 包组织（对齐 Hunav_isaac_wrapper）

| Hunav_isaac_wrapper | autosim |
|---------------------|---------|
| `hunav_manager.py` | `backends/isaac/hunav/manager.py` |
| `world_builder.py` | `backends/isaac/hunav/world_builder.py` |
| `animation_utils.py` | `backends/isaac/hunav/animation_utils.py` |
| `teleop_hunav_sim.py` | Runner + `/cmd_vel`（已有） |
| `scenarios/*.yaml` | `config/scenarios/hunav/` |
| ROS `hunav_msgs` | **automsgs** `hunav_msgs` |

引擎无关逻辑（编解码 / SFM / Autolink service）在 `scene/hunav/`。

---

## 4. 配置

```yaml
habitat:
  hunav:
    service: true          # Runner 注册 /compute_agents
    publish_agents: true   # 发布 /hunav/agents
    # manager: true        # Isaac：HuNavManager 驱动 People（own_motion）
    # own_motion: true     # true 时跳过 Traffic.pedestrian.step
    # agents: config/scenarios/hunav/warehouse_agents.yaml
  scenario:
    pedestrian:
      enabled: true
      social: true
      # 原生或 HuNav 布局均可：
      agents: config/scenarios/hunav/warehouse_agents.yaml
```

HuNav YAML（`hunav_loader.ros__parameters` + `global_goals`）由 `scene/hunav/loader.py` 自动展开。

`warmup_map` 若重建 Traffic，Runner 会 `rebind_hunav_after_traffic`，保证 service / manager 仍指向新的 Pedestrian。

---

## 5. 代码入口

```python
from autosim.scene.hunav import ComputeAgentsEngine, load_hunav_scenario
from autosim.backends.isaac.hunav import HuNavManager, WorldBuilder

roster = load_hunav_scenario("config/scenarios/hunav/warehouse_agents.yaml")
engine = ComputeAgentsEngine()
# Isaac Kit 内：
# manager = HuNavManager(stage, config_file=..., robot_pose_fn=...)
# manager.send_agents(dt=0.02)
```

重建消息：在 `automsgs` 目录执行既有 `tools/generate_messages.sh` / colcon / Bazel 生成流程。

---

## 6. 测试

```bash
# 单元测试（SpaceHero）
export PYTHONPATH=/usr/local/lib/python:/workspace/autonomy/src/autonomy/autosim
cd /workspace/autonomy/src/autonomy/autosim
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 python3 -m pytest tests/test_hunav.py -q

# fake 冒烟（服务 + /hunav/agents，无需 Isaac）
export PYTHONPATH=/usr/local/python:/usr/local/lib/python:/workspace/autonomy/src/autonomy/autosim
python3 scripts/smoke_hunav.py --config config/fake.yaml --steps 30

# Isaac + HuNavManager（需 Isaac python 3.11 + install/.../python/autolink）
export AUTOSIM_ISAAC_HEADLESS=1
export PYTHONPATH=/workspace/autonomy/install/autonomy/python:/usr/local/lib/python:/workspace/autonomy/src/autonomy/autosim
/workspace/isaaclab/_isaac_sim/python.sh scripts/smoke_hunav.py \
  --config config/isaac.yaml --manager --steps 15
```
