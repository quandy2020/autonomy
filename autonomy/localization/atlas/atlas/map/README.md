# Atlas dense mapping

This directory contains the ROS-independent dense mapping pipeline used by
Atlas. AutoMsgs conversion and Autolink publication remain in
`atlas/message_adapter.*` and `atlas/atlas_node.*`.

## Data flow

1. `MapBuilder` retains recent tracked images until ORB-SLAM promotes their
   timestamps to keyframes.
2. RGB-D keyframes are back-projected directly. Rectified stereo keyframes use
   StereoSGBM, while calibrated pinhole and Kannala-Brandt8 pairs are rectified
   first. Monocular keyframes use temporal stereo between two optimized
   keyframe poses. The same map paths are used by the corresponding inertial
   sensor modes.
3. `MappingPipeline` projects and pre-voxel-filters each local cloud on a
   bounded worker queue, so tracking callbacks do not perform dense work.
4. `DenseMap` incrementally fuses colored voxels. `OccupancyGridMap` updates a
   sparse two-dimensional log-odds grid with endpoint hits and Bresenham free
   rays.
5. Recent camera-frame clouds remain in RAM and older history is archived in a
   per-instance disk cache. A loop closure or bundle-adjustment pose change
   transactionally rebuilds dense, navigation and occupancy maps from the full
   available history; the previous complete snapshot remains active if loading
   or rebuilding fails.

## Supported inputs

- RGB-D and RGB-D + IMU use the registered depth image directly.
- Stereo and stereo + IMU support rectified pairs, raw pinhole pairs, and
  Kannala-Brandt8 fisheye pairs using the second camera calibration and rig
  transform.
- Monocular and monocular + IMU use optimized adjacent keyframes as temporal
  stereo pairs. A loop-closure scale change re-triangulates the cached pairs.

Pure monocular SLAM has no observable absolute scale. Its point cloud and grid
therefore use Atlas' relative map scale. Monocular-inertial mapping becomes
metric after inertial scale initialization.

## Performance and bounds

The defaults use 4-pixel depth decimation, 5 cm voxels and grid cells, a
drop-oldest input queue (the Atlas integration enforces at least four frames),
5 m maximum depth, and hard limits for local points, global voxels, grid cells,
snapshot area and cached keyframes. Duplicate frame identifiers are rejected,
and optimized poses are indexed by frame identifier so long trajectories do
not scan their full history on every frame. Frames beyond
`MaximumCachedFrames` are archived under a unique child of
`HistoryCacheDirectory`, subject to `MaximumHistoryCacheBytes`. Global
snapshots are immutable and
versioned; publishers copy an already-built snapshot instead of rebuilding
maps.

Optional values can be added to the Atlas camera YAML:

```yaml
Atlas.Map.RangeMin: 0.2
Atlas.Map.RangeMax: 5.0
Atlas.Map.PixelStride: 4
Atlas.Map.VoxelSize: 0.05
Atlas.Map.GridResolution: 0.05
Atlas.Map.MinHeight: -0.5
Atlas.Map.MaxHeight: 2.0
Atlas.Map.MaximumExtent: 250.0
Atlas.Map.MinimumTranslation: 0.10
Atlas.Map.MinimumRotationDegrees: 5.0
Atlas.Map.SnapshotPeriod: 1.0
Atlas.Map.MinimumTemporalBaseline: 0.02
Atlas.Map.MaximumTemporalBaseline: 2.0
Atlas.Map.QueueCapacity: 4
Atlas.Map.MaximumCachedFrames: 400
Atlas.Map.MaximumHistoryCacheBytes: 8589934592
Atlas.Map.HistoryCacheDirectory: "/tmp"
Atlas.Map.MaximumPointsPerFrame: 150000
Atlas.Map.MaximumVoxels: 1000000
Atlas.Map.MaximumNavigationVoxels: 1500000
Atlas.Map.NavigationRadius: 30.0
Atlas.Map.NavigationRecenterDistance: 5.0
Atlas.Map.MaximumCells: 4000000
Atlas.Map.MaximumSnapshotCells: 2500000
Atlas.Map.SensorHeight: 1.0
Atlas.Map.EstimateGroundPlane: 1
Atlas.Map.NavigationMinHeight: -1.0
Atlas.Map.NavigationMaxHeight: 1.0
Atlas.Map.RobotRadius: 0.30
Atlas.Map.InflationRadius: 0.60
Atlas.Map.InflationCostScaling: 5.0
```

`HistoryCacheDirectory` must name an existing writable parent directory. A
value of zero for `MaximumHistoryCacheBytes` disables disk-backed history; a
full-history loop rebuild is then skipped once RAM history is incomplete.

For ground robots, configure the physical `SensorHeight`. Without it Atlas uses
a conservative sensor-relative fallback: points above `MaxHeight` remain
unknown, only points below `MinHeight` may clear free space, and the navigation
cloud is filtered by `NavigationMinHeight..NavigationMaxHeight` relative to the
sensor. The fallback is safe but is not a ground-relative production setup.

Set `NavigationRadius` above zero to keep the navigation cloud in a rolling
sphere around the robot. It is re-centered only after
`NavigationRecenterDistance` of travel; global dense and occupancy maps remain
unchanged. Zero disables the rolling window for backward compatibility.

Inflation is applied only to published known-free cells. Occupied cells remain
100, unknown cells remain -1, and costs 1..99 decay outside `RobotRadius` until
`InflationRadius`; the internal log-odds map is unchanged. Set all three
inflation values to zero to disable it.

Defaults follow the RTAB-Map local-grid/global-map architecture and common
probability parameters, but this implementation is independent and contains
no copied RTAB-Map source.
