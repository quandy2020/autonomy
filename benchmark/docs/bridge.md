# Autolink / autonomy.control bridge

## AutolinkBridge

`benchmark.bridge.AutolinkBridge` is the Autolink transport (no ROS runtime):

| Direction | Channel | Message |
|-----------|---------|---------|
| publish | `/cmd_vel` | `TwistStamped` |
| subscribe | `/odom` | `Odometry` → `Pose2D` |
| subscribe | `/scan` | `LaserScan` (optional) |

When `autolink` / `automsgs` are missing, it uses a null transport so offline
evaluation still works.

## Isaac / Habitat sidecar

`SidecarSimulator` (used by `isaac` / `habitat`):

1. Optionally launch `autosim/scripts/run.sh sim`
2. Connect AutolinkBridge and publish each planner twist to `/cmd_vel`
3. Keep a kinematic twin for Arena metrics; optionally sync pose from `/odom`

```bash
benchmark eval --set simulator.name=isaac \
  --set simulator.launch_autosim=true \
  --set simulator.autolink=true
```

## autonomy.control MPPI

`planner.name=mppi` → `AutonomyMppiPlanner`:

- Starts `autonomy.control` when the binary is found (`benchmark doctor`)
- Mirrors local MPC / sampling twists onto `/cmd_vel`
- Falls back to local MPC when control/autolink are absent
