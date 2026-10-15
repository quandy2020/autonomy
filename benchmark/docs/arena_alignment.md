# Arena-Rosnav 能力对齐

对照 [Arena-Rosnav](https://github.com/Arena-Rosnav) / Arena 3.0–5.0 功能面。

| Arena 能力 | 本仓库 | 状态 |
|------------|--------|------|
| Flatland 2D | `simulator.name=flatland` / `fake` | ✅ |
| Gazebo | `gazebo`（sidecar + twin） | ✅ 适配 |
| Unity | `unity` / `habitat`（RGB-D sidecar） | ✅ 适配 |
| Isaac Sim / Gym | `isaac`（autosim + Isaac Lab） | ✅ |
| HIL / 实机 | `hil` + autodriver | ✅（额外） |
| Task: scenario / random / parametrized / guided / explore | `task.mode=…` | ✅ |
| Task: zones / staged / dynamic / benchmark | 同上 | ✅ |
| Scenario JSON | `configs/scenarios/*.json` + `scenario` mode | ✅ |
| SFM: Pedsim / Helbing | `humans.name=social_force` / `pedsim` | ✅ |
| PySocialForce + groups | `pysocialforce` | ✅ |
| ORCA / RVO2 | `orca` / `rvo2` | ✅ |
| Evacuation / Spinny | `evacuation` / `spinny` | ✅ |
| HuNavSim states | `hunav` | ✅ |
| LightSFM / Deep Social | `lightsfm` / `deep_social` | ✅ |
| Crowd flow | `crowd_flow` | ✅ |
| Planners: DWA / TEB / MPC | ✅ | ✅ |
| ROSNavRL | `rosnav_rl` / `policy` + ONNX | ✅ |
| CADRL / CrowdNav / RLCA / SARL / Applr / Dragon / Trail | 同名 planner（ONNX 或 reactive） | ✅ 接口 |
| Move Base Flex hybrid | `hybrid` / `mbf` | ✅ |
| MPPI / NMPC | `mppi` / `nmpc` / autonomy.control | ✅ |
| World gen: maze / barn / canteen / office / industrial | `generate-world` + `dynamic` mode | ✅ |
| Prebuilt worlds | hospital / warehouse / canteen / office / posts | ✅ |
| Robots: turtlebot / jackal / husky / waffle / ridgeback / tiago / dingo / youbot / go1 / cob4 | `configs/robots/` | ✅ |
| Arena Evaluation: safety / efficiency / smoothness | ✅ | ✅ |
| Social: private zone / facing / seen-by | `evaluation.social` | ✅ |
| Multi-robot | `benchmark multi-robot` | ✅ |
| SB3 / curriculum / Optuna | `train` / `tune` | ✅ |
| Dataset collect / replay | `collect` / `replay` | ✅（额外） |
| SLAM / perception tasks | ✅（额外） | ✅ |
| HTTP API | `benchmark serve` | ✅ |
| Arena-Gen LLM world | `benchmark generate-world --prompt "…"`（关键词映射，无 LLM） | ✅ lite |
| 原生 Gazebo/Unity 二进制 | 走 autosim sidecar，非原生引擎 | ⏳ 适配层 |
| Sidecar scan→obs / odom sync | `inject_scan` + `sync_odom`（habitat/isaac/gazebo/unity） | ✅ |

## 仿真器映射

| Arena | benchmark |
|-------|-----------|
| Flatland | `flatland` / `fake` |
| Gazebo | `gazebo` |
| Unity | `unity` / `habitat` |
| Isaac | `isaac` |

## 快速验证

```bash
benchmark list
benchmark run --mode benchmark --set task.episodes=8
benchmark run --mode dynamic --set task.map=canteen
benchmark eval --set humans.name=hunav --set planner.name=cadrl --episodes 3
benchmark run --mode scenario --set task.scenario=configs/scenarios/demo_challenge.json
```
