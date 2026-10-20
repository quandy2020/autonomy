# hunav_msgs（automsgs）

HuNavSim [`hunav_msgs`](https://github.com/robotics-upo/hunav_sim) 的 Autolink / Protobuf 对齐层，供 **autosim ↔ Isaac Sim** 行人社会导航联调。

| Proto | 对应 ROS msg/srv |
|-------|------------------|
| `AgentBehavior` | `hunav_msgs/AgentBehavior` |
| `Agent` | `hunav_msgs/Agent` |
| `Agents` | `hunav_msgs/Agents` |
| `srvs/ComputeAgents` | `/compute_agents` |
| `srvs/ComputeAgent` | `/compute_agent` |
| `srvs/MoveAgent` | `/move_agent` |

## 关系

```text
Isaac (USD / People)                 Autolink                    autosim SFM
  pack Agents + robot  ──/compute_agents──►  Pedestrian / Force
  apply updated Agents ◄── Response ───────  step(dt)
```

枚举取值与 HuNav 一致（`PERSON=1`、`BEH_REGULAR=1` …），便于对照场景 YAML 与 Behavior Tree。

实现与场景组织见 `autosim/backends/isaac/hunav/`、`autosim/docs/12_hunav_isaac.md`。
