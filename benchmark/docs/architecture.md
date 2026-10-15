# Architecture

Inspired by [Arena-Rosnav](https://github.com/Arena-Rosnav) (Arena 5.0): simulators × planners × task modes × evaluation × RL training, adapted to the autonomy monorepo.

## Layers

| Layer | Package | Arena analogue |
|-------|---------|----------------|
| CLI | `benchmark.cli` | arena launch / scripts |
| Task modes | `benchmark.tasks.modes` | task-generator |
| Episode runner | `benchmark.tasks.episode_runner` | arena core loop |
| Simulators | `benchmark.simulators` | flatland / gazebo / unity / isaac |
| Planning | `benchmark.planning` | arena planners |
| Humans | `benchmark.humans.social_force` | social force / huNav |
| Worlds | `benchmark.worlds.generators` | dynamic map / maze |
| Evaluation | `benchmark.evaluation` | arena-evaluation |
| RL | `benchmark.rl` | rosnav-rl / Arena-Training |
| Bridge | `benchmark.bridge` | ROS bridge → autolink |

## Navigate data flow

```
YAML (task.mode + planner + sim)
  → create_mode() + make_planner() + create_simulator()
  → run_episode()  [trajectory points]
  → ArenaEvaluation.summarize()
  → data/eval_results/
```
