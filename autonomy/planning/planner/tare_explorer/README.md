# Native TARE integration

This directory is the point-to-point planning compatibility façade for upstream
TARE. It does not own exploration state, frontier selection, a keypose graph, or
an exploration thread.

## Ownership

- `autonomy/task/exploration` owns the asynchronous exploration lifecycle,
  measured coverage ledger, frontier/surface requirements, rolling viewpoint
  field, hierarchical selection, return budgeting, and observation receipts.
- Vista `PathProvider` is the only owner of graph edges, route feasibility,
  directed route costs, and retained route geometry.
- `tare_explorer_planner.*` preserves the ordinary `GlobalPlanner` contract for
  an exact requested start and goal. TARE planner aliases share the configured
  Vista planner selected by `vista_planner_id`; they do not create another
  graph.
- Public transport remains existing automsgs/protobuf types. This integration
  defines no duplicate exploration path message.

Every production header in this directory declares one class. The existing
parent CMake project builds these sources; there is intentionally no local CMake
project and no Bazel target.

## Shared option ownership

`ExplorationOptions` is loaded by the native sensor platform and translated once
by `ApplySharedExplorationOptions`:

- keypose acquisition spacing, the independent measured home-arrival threshold,
  and route lookahead/line-of-sight policy belong to `ExplorationSession`; the
  RGB-D platform converts the configured metric viewpoint-grid extent to
  measured-volume cells before allocating its bounded rolling field;
- grid scale, connected frontier minimum size, bounded local cover-set attempts,
  viewpoint spacing/yaw count, robot collision radius, frontier scheduling, and
  measured-motion hysteresis belong to `HierarchicalCoverageStrategy`;
- the configured small-node threshold selects exact directed Held-Karp ordering;
  larger bounded sets use greedy ordering with optional directed two-opt, while
  every edge, asymmetric cost, feasible prefix, and return leg still comes from
  immutable Vista evidence;
- `keypose_connect_dist_m` and FAR graph/edge controls are deliberately not
  translated because acquisition keyposes own no edges and Vista is the sole
  route owner; non-graph FAR behaviors without a native equivalent fail closed;
- completion transitions directly to a Vista-owned return route, so the unsafe
  upstream straight-line rush-home shortcut is neither needed nor reproduced;
- camera/depth calibration and frames are checked against their native owners;
  a conservative shared sensor range becomes the camera visibility limit;
  `use_terrain_height` reuses only measured per-cell floor heights and the
  bounded measured RGB-D volume for 3-D body clearance. Viewpoints and routes
  remain `Pose2d`, and Vista remains the sole route owner; missing measured
  volume support, non-RGB-D terrain sources, or incomplete body collision
  geometry fail explicitly rather than falling back to predicted heights;
  unsupported LiDAR, point-cloud, prior-map, planner-costmap, and task lifecycle
  settings are rejected when non-default rather than ignored;
- file-based boundary/no-go settings are not used as hidden global geometry.
  Runtime `geometry_msgs::Polygon` task geometry is authoritative: allowed and
  no-go polygons are validated as finite, simple, and nondegenerate, computed
  `Bounds2d` values are only a broad phase, and frontier, measured-surface,
  viewpoint, external coverage, and adapted execution candidates use exact
  polygon membership. Every explicit and implicit segment of a successful
  Vista route is checked without replacing its path, cost, or evidence. The
  allowed boundary is inclusive; every no-go polygon is a closed exclusion,
  including its edges and vertices.

The native RGB-D platform requires `exploration_options_json`, so generated TARE
settings cannot silently fall back to unrelated native defaults.

## Evidence and execution semantics

Completion is based only on measured coverage evidence. Sparse 3-D volume
omissions remain unknown. Production extraction creates surface requirements
only for occupied voxel faces adjacent to explicitly measured free voxels.
When a compatible measured volume exists, frontier requirements are omitted
in-bounds horizontal neighbors of explicitly measured free voxels; volume caps
above and below measured free space are not ground-exploration frontiers. The
requirements are clustered in 3-D by physical center separation, including
vertical and diagonal links within `frontier_cluster_tolerance_m` (one meter by
default), rather than by a fixed voxel-index stencil. The planar grid is used
only when no volume exists. Both paths enforce the configured metric tolerance
and connected-component minimum.

The first admitted Vista route remains immutable evidence. Arc-length lookahead
may execute an intermediate waypoint, after which Vista replans from the
measured pose to the same exact final observation pose. Intermediate arrival
never records observation; final position and yaw plus actual sensor evidence
are required.

## Build and test

Build inside the project environment without Bazel:

```sh
source /opt/ros/humble/setup.bash
cd /workspace/autonomy
colcon build --packages-up-to autonomy --cmake-args \
  -DBUILD_TEST=ON -DAUTONOMY_BUILD_EXPLORATION_TESTS=ON
```
