# autonomy-benchmark

对标 [Arena-Rosnav](https://github.com/Arena-Rosnav) 的一站式实验与评测平台：导航 / SLAM / 感知 / 数据采集与回灌 / HAL 在环。

配置驱动、插件注册；仿真走 [autosim](../autosim)，栈内对接 `autonomy` / `autolink` / `automsgs` / `autodriver` / `automanip`。

## Arena 能力对照

| Arena-Rosnav | 本仓库 `benchmark` |
|--------------|-------------------|
| Flatland / Gazebo / Unity / Isaac | `fake` / `habitat` / `isaac` / `hil` |
| Task modes | `scenario` / `random` / `parametrized` / `guided` / `explore` |
| Planners | `pure_pursuit` / `dwa` / `mpc` / `teb` / `policy` / `rosnav_rl` / `hybrid` / `mppi` |
| Social force / crowd | `humans.social_force` / `humans.crowd_flow`（接入 FakeSimulator） |
| Dynamic map / maze | `benchmark generate-world` |
| Arena Evaluation | `benchmark eval` → safety / efficiency / smoothness / status + HTML |
| ROSNavRL + SB3 | `benchmark train` / `tune` / `--export` |
| Multi-robot | `benchmark multi-robot` |
| Data collect / replay | JSONL + 可选 LeRobot；`replay` → `/cmd_vel` |
| HTTP API | `benchmark serve`（`.[api]`） |

## 快速开始

```bash
cd src/autonomy/benchmark
pip install -e ".[dev]"

benchmark list
benchmark doctor
bash scripts/smoke_e2e.sh          # 端到端冒烟

benchmark run --mode random --set task.episodes=5 \
  --set humans.name=social_force --set humans.behavior=commuter
benchmark eval --episodes 10 --set planner.name=dwa
benchmark train --curriculum --episodes 30 --export data/checkpoints/policy
benchmark multi-robot --robots 3 --planner hybrid
benchmark generate-world maze --seed 3 -o configs/worlds/maze_gen.yaml
python scripts/benchmark_planners.py
python scripts/export_onnx.py
```

### 任务一览

```bash
benchmark run --task navigate|slam|perception|collect|replay|hil
benchmark eval --episodes 5
benchmark train --curriculum --export data/checkpoints/policy
benchmark tune --trials 5          # 需 optuna
benchmark serve --port 8088        # 需 pip install -e '.[api]'
```

### Isaac sidecar

```bash
bash scripts/run_isaac.sh
# 或
benchmark eval --set simulator.name=isaac \
  --set simulator.launch_autosim=true --set simulator.autolink=true
```

## 行人模型

```yaml
# configs/default.yaml
humans:
  name: social_force   # or crowd_flow
  enabled: true
  behavior: commuter   # leisure | commuter | rush
  preferred_speed: 1.2
```

## 目录

```
benchmark/
├── configs/     # sim / planner / robot / world / task / humans / eval
├── src/benchmark/
│   ├── core/ simulators/ tasks/ planning/ humans/ worlds/
│   ├── evaluation/ rl/ bridge/ data/ api/ cli/
├── scripts/     # smoke_e2e · benchmark_planners · export_onnx · run_isaac
├── tests/
└── docs/
```

更多：[architecture](docs/architecture.md) · [tasks](docs/tasks.md) · [bridge](docs/bridge.md) · [Arena-Rosnav](https://github.com/Arena-Rosnav)
