# Task behavior trees

Each app under `autonomy/task/*` owns a **TaskBtProfile**:

| Task | Default XML | Alternate XML | Tick (ms) |
|------|-------------|---------------|-----------|
| navigation | `navigation/navigate_to_pose.xml` | `navigation/navigate_through_poses.xml` | 10 |
| tracking | `tracking/follow_target.xml` | `tracking/follow_person.xml` | 20 |
| teleop | `teleop/teleop.xml` | — | 50 |
| charging | `charging/dock.xml` | — | 10 |
| mapping | `mapping/map_ops.xml` | — | 100 |
| localization | `localization/localization.xml` | — | 100 |

Configure via `config/task/task_options.lua` or `TaskServerOptions.behavior_trees`.

Goal-level override: `NavigationGoal.plugins.behavior_tree`.
