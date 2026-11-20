# v2.124 native RGB-D exploration validation

This directory contains the offline evidence from the AutoSim run with
Habitat `17DRP5sb8fy`, Atlas RGB-D SLAM, the native task dispatcher, Vista,
SanD, and the MPPI tracker.

- `report.json`: validator output, message counts, and raw execution hashes.
- `task.log`: native task and controller log.
- `validation-summary.json`: pass thresholds, GT motion, and map metrics.
- `native-build.log`: final build of control, planning, task, and task app targets.
- `ctest-43.log`: the complete native CTest selection; 43/43 pass.
- `ctest-exploration-7.log`: focused exploration/control/BT regression; 7/7 pass.
- `comparison/map-comparison.json`: occupancy and coverage metrics after an
  explicit `native_exploration_map` -> `map` projection.
- `comparison/gt_from_native.transform.pb`: transform derived only from the
  first recorded GT/native pose pair.
- `comparison/*_on_gt.pb`: projected native occupancy and coverage ledgers on
  the 0.05 m GT lattice.
- `gt_pose_first.pb`, `gt_pose_last.pb`, `native_occupancy_*.pb`,
  `measured_coverage_*.pb`, and `map_grid_last.pb`: original recorded inputs.

The run used a 140 s exploration budget and ended at that partial budget. It
is evidence of system feasibility, movement, SLAM/map integration, coverage
growth, and the command-safety path. It is not a claim of complete-room
exploration. The final feedback status therefore remains a non-success
outcome after the budget stop; `validation-summary.json` records the explicit
feasibility thresholds separately.

At the final snapshot, the native occupancy map grew from 453 to 1799 known
cells. The measured coverage ledger grew from 131 to 1223 requirement cells;
1162 were confirmed. The first-to-last GT translation was 2.105 m. The
offline projection measured 43.35% known coverage of the start-connected
reachable GT free domain, 96.66% confirmed coverage among requirements, and
0.33% reachable false-occupied cells. The 0.105 m radius matches the
Turtlebot3 Burger collision envelope used by this run.

The official frozen DeFM model and parity fixture were copied into the isolated
container from the host `/tmp/autonomy-defm-v2.47` before the final test run.
Their SHA-256 values are `334320ee...99c4` and `b8b5d4ea...50bf`; the DeFM
encoder test passed.
