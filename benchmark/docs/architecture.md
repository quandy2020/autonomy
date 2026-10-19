# Architecture

Inspired by [Arena-Rosnav](https://github.com/Arena-Rosnav) (Arena 5.0): simulators × planners × task modes × evaluation, adapted to the autonomy monorepo.

## Layers

| Layer | Package | Arena analogue |
|-------|---------|----------------|
| Common | `benchmark.common` | config / registry / types |
| Entry | `benchmark.main` | arena launch / scripts |
| Tasks | `benchmark.tasks` (+ `modes` / `planning`) | task-generator + arena planners |
| Episode runner | `benchmark.tasks.episode_runner` | arena core loop |
| Simulators + worlds | `benchmark.simulators` (+ `worlds` / `humans`) | flatland / gazebo / unity / isaac + SFM |
| Evaluation + datasets | `benchmark.evaluation` | arena-evaluation |
| Bridge | `benchmark.bridge` | Autolink（无 ROS） |
| Dashboard | `benchmark.dashboard` | — |

## Navigate data flow

```
YAML (task.mode + planner + sim)
  → create_mode() + make_planner() + create_simulator()
  → run_episode()  [trajectory points]
  → ArenaEvaluation.summarize()
  → data/eval_results/
```
