-- Cartographer 2D mapping for autosim (turtlebot3 + Habitat).
-- Standard SLAM TF tree:
--   map → odom              (Cartographer, dynamic correction)
--   odom → base_link        (autosim only; no map→odom from sim)
--   base_link → …           (URDF mounts)
-- Requires habitat.mode=slam (publish_map_odom=false, odom→base_link).

include "map_builder.lua"
include "trajectory_builder.lua"

options = {
  map_builder = MAP_BUILDER,
  trajectory_builder = TRAJECTORY_BUILDER,
  map_frame = "map",
  tracking_frame = "base_link",
  published_frame = "odom",
  odom_frame = "odom",
  provide_odom_frame = false,
  publish_frame_projected_to_2d = true,
  use_odometry = true,
  use_nav_sat = false,
  use_landmarks = false,
  num_laser_scans = 1,
  laser_scan_topics = { "/scan" },
  num_multi_echo_laser_scans = 0,
  num_subdivisions_per_laser_scan = 1,
  num_point_clouds = 0,
  imu_topic = "/imu",
  odometry_topic = "/odom",
  lookup_transform_timeout_sec = 0.5,
  submap_publish_period_sec = 0.3,
  pose_publish_period_sec = 5e-2,
  trajectory_publish_period_sec = 30e-3,
  rangefinder_sampling_ratio = 1.,
  odometry_sampling_ratio = 1.,
  fixed_frame_pose_sampling_ratio = 1.,
  imu_sampling_ratio = 1.,
  landmarks_sampling_ratio = 1.,
  ignore_out_of_order = true,
  publish_to_tf = true,
  publish_tracked_pose = true,
  publish_occupancy_grid = true,
  occupancy_grid_publish_period_sec = 1.0,
  occupancy_grid_resolution = 0.05,
}

MAP_BUILDER.use_trajectory_builder_2d = true
MAP_BUILDER.use_trajectory_builder_3d = false

TRAJECTORY_BUILDER_2D.min_range = 0.12
-- Match autosim lidar range_max; short max_range invents miss rays.
TRAJECTORY_BUILDER_2D.max_range = 30.
TRAJECTORY_BUILDER_2D.missing_data_ray_length = 5.
-- One full 360° scan per local SLAM step (not backpack multi-echo).
TRAJECTORY_BUILDER_2D.num_accumulated_range_data = 1
TRAJECTORY_BUILDER_2D.use_imu_data = false
-- Online CSM for a coarse prior; ceres weights at Cartographer defaults so
-- laser can correct GT-glued odom instead of painting walls at wrong poses.
TRAJECTORY_BUILDER_2D.use_online_correlative_scan_matching = true
TRAJECTORY_BUILDER_2D.ceres_scan_matcher.translation_weight = 10.
TRAJECTORY_BUILDER_2D.ceres_scan_matcher.rotation_weight = 40.
TRAJECTORY_BUILDER_2D.submaps.num_range_data = 90

POSE_GRAPH.optimize_every_n_nodes = 90
POSE_GRAPH.constraint_builder.min_score = 0.55
-- Default odometry weights (1e5); 1e7 blocked loop closure from fixing drift.
POSE_GRAPH.optimization_problem.odometry_translation_weight = 1e5
POSE_GRAPH.optimization_problem.odometry_rotation_weight = 1e5
-- Do NOT enable overlapping_submaps_trimmer_2d: trimming old submaps causes
-- double walls / ghosting when revisiting corridors.

return options
