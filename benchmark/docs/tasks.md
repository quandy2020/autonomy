# Tasks & Arena modes

## High-level tasks

| Task | Role |
|------|------|
| navigate | Goal / waypoint / explore episodes on a simulator |
| slam | ATE / RPE / map coverage vs GT (noisy odom or `/odom`) |
| perception | Lidar clustering precision / recall vs obstacle GT |
| collect | JSONL (+ optional LeRobot) demonstration dump |
| replay | Offline replay onto autolink `/cmd_vel` |
| hil | Hardware-in-the-loop with safety clamp + twin |

## Arena task modes (`task.mode`)

Aligned with [Arena-Rosnav task-generator](https://github.com/Arena-Rosnav):

| Mode | Robots | Obstacles | Notes |
|------|--------|-----------|-------|
| scenario | ✓ | ✓ | Load fixed start/goal/density |
| random | ✓ | ✓ | Sample poses and counts |
| parametrized | ✓ | ✓ | Difficulty curriculum |
| guided | ✓ | ✓ | Waypoint sequence |
| explore | ✓ | ✓ | Coverage objective |

## Evaluation

`benchmark eval` runs the Arena Evaluation suite (safety / efficiency / smoothness / status)
and writes JSON / Markdown / HTML under `data/eval_results/`.
