# autonomy-benchmark

对标 [Arena-Rosnav](https://github.com/Arena-Rosnav)（Arena 3.0–5.0）的一站式实验与评测平台：导航 / SLAM / 感知 / 数据采集与回灌 / HAL 在环。

配置驱动、插件注册；仿真走 [autosim](../autosim)，栈内对接 `autonomy` / `autolink` / `automsgs` / `autodriver` / `automanip`。

完整对照表见 [docs/arena_alignment.md](docs/arena_alignment.md)。

## Arena 能力对照

| Arena-Rosnav | 本仓库 `benchmark` |
|--------------|-------------------|
| Flatland / Gazebo / Unity / Isaac | `flatland`·`fake` / `gazebo` / `unity`·`habitat` / `isaac` (+ `hil`) |
| Task modes | `scenario` `random` `parametrized` `guided` `explore` `zones` `staged` `dynamic` `benchmark` |
| Planners | `dwa` `teb` `mpc` `mppi` `rosnav_rl` `cadrl` `crowdnav` `rlca` `sarl` `applr` `dragon` `trail` `hybrid`/`mbf` … |
| Social force / HuNav | `social_force` `pysocialforce` `orca` `rvo2` `evacuation` `spinny` `hunav` `lightsfm` `deep_social` `crowd_flow` |
| Dynamic map / maze | `benchmark generate-world` + `mode=dynamic`（barn/canteen/office/industrial…） |
| Arena Evaluation | safety / efficiency / smoothness / **social**（private zone · facing · seen-by）+ HTML |
| ROSNavRL + SB3 | `benchmark train` / `tune` / `--export` |
| Multi-robot | `benchmark multi-robot` |
| Robots | turtlebot3 · waffle · jackal · husky · ridgeback · tiago · dingo · youbot · go1 · cob4 |
| Worlds | hospital · canteen · warehouse · office · posts · maze |
| Data / API | collect·replay · `benchmark serve` |

## 快速开始

```bash
cd src/autonomy/benchmark
pip install -e ".[dev]"

benchmark list
benchmark doctor
bash scripts/smoke_e2e.sh

# Arena 风格挑战
benchmark run --mode benchmark --set task.episodes=8
benchmark run --mode dynamic --set task.map=canteen --set task.episodes=2
benchmark run --mode scenario --set task.scenario=configs/scenarios/demo_challenge.json

# 社交行人 + DRL 规划器名
benchmark eval --episodes 5 --set humans.name=hunav --set planner.name=cadrl
benchmark eval --set humans.name=pysocialforce --set planner.name=crowdnav

# Isaac / Habitat / Gazebo sidecar
bash scripts/run_isaac.sh
bash scripts/run_habitat.sh          # LAUNCH=true 时拉起 autosim
bash scripts/run_gazebo.sh

# Arena-Gen lite（关键词 → 世界）
benchmark generate-world --prompt "hard hospital level 3 with 12 pedestrians" -o configs/worlds/gen_hospital.yaml
```

### 任务一览

```bash
benchmark run --task navigate|slam|perception|collect|replay|hil
benchmark eval --episodes 5
benchmark train --curriculum --export data/checkpoints/policy
benchmark tune --trials 5
benchmark serve --port 8088        # pip install -e '.[api]'
benchmark multi-robot --robots 3 --planner hybrid
benchmark generate-world maze --seed 3 -o configs/worlds/maze_gen.yaml
```

## 行人模型

```yaml
humans:
  name: hunav          # social_force | pysocialforce | orca | evacuation | …
  enabled: true
  behavior: commuter   # leisure | commuter | rush
  preferred_speed: 1.2
```

## 目录

```
benchmark/
├── configs/     # sim / planner / robot / world / task / humans / scenarios / eval
├── src/benchmark/
│   ├── core/ simulators/ tasks/ planning/ humans/ worlds/
│   ├── evaluation/ rl/ bridge/ data/ api/ cli/
├── scripts/     # smoke_e2e · benchmark_planners · export_onnx · run_isaac
├── tests/
└── docs/        # architecture · tasks · bridge · arena_alignment
```

更多：[architecture](docs/architecture.md) · [tasks](docs/tasks.md) · [bridge](docs/bridge.md) · [arena_alignment](docs/arena_alignment.md) · [Arena-Rosnav](https://github.com/Arena-Rosnav)
