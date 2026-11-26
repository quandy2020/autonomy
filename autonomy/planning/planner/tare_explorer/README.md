# Native TARE adapter

This directory adapts the planning structure of
[`tare_planner`](../../../../../../ros2/autonomous_exploration_development_environment/src/tare_planner)
to autonomy without embedding a ROS 2 node.

## Semantic boundary

Upstream TARE is a continuous 3-D exploration coordinator. It consumes point
clouds, odometry and boundaries, maintains viewpoints/keyposes/coverage, solves
global and local tours, and publishes a look-ahead waypoint. It is **not** a
normal `(start, goal) -> Path` planner.

The native adapter therefore has two explicit APIs:

- `TareExplorerCore::Plan` / `TareExplorerPlanner::CreateExplorationPlan`:
  detects and clusters 2-D occupancy frontiers, selects global subspaces by
  information gain and travel cost, and stitches collision-free local A* paths.
  A home route is produced only when exploration is complete.
- `TareExplorerPlanner::CreatePlan`: preserves autonomy's strict
  `GlobalPlanner` start-to-explicit-goal contract. It reuses only the bounded
  local route core and never silently replaces the requested goal with a
  frontier.

The adapter deliberately does not import `rclcpp`, ROS messages,
`message_filters`, PCL visualization, or OR-Tools. It snapshots the shared
`Costmap2D` under its mutex and uses STL-only deterministic algorithms.

## Limitations

A 2-D costmap cannot reproduce upstream TARE's terrain height, registered scan,
3-D sensor coverage, keypose graph, or point-cloud line-of-sight evidence. This
module adapts TARE's hierarchical exploration flow, not those unavailable 3-D
semantics. Those inputs should be added through a native exploration data
interface rather than by restoring ROS publishers/subscribers in this library.
