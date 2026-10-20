# isaac/hunav

[Hunav_isaac_wrapper](https://github.com/robotics-upo/Hunav_isaac_wrapper) 的 Autolink 移植（实现与上游对齐）。

| 模块 | 原包 |
|------|------|
| `manager.py` | `hunav_manager.py`（SkelRoot 生成 + AnimGraph + PhysX 射线 + `/compute_agents`） |
| `world_builder.py` | `world_builder.py` |
| `animation_utils.py` | `animation_utils.py`（retarget / AnimationGraph / speed） |

与上游差异：

- ROS 2 `rclpy` / `hunav_msgs` → **Autolink** + `automsgs` `hunav_msgs`
- `/compute_agents` → 进程内 `ComputeAgentsEngine` 或远程 Autolink client
- 角色资源优先 `habitat.isaac.traffic.character_root` / `animation_root`，否则 Nucleus `Isaac/People`

`habitat.hunav.manager: true` 时，`HuNavManager` 禁用 `PeopleSync` 镜像，自行在 `{World\|Root}/Characters` 下生成角色。

消息与服务：`automsgs/msgs/hunav_msgs`、`automsgs/srvs/compute_agents`。  
文档：`docs/12_hunav_isaac.md`。
