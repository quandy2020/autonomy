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
  pose_publish_period_sec = 20e-3,
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
  save_map_image = true,
  map_image_save_period_sec = 10.0,
  map_image_save_directory = "data/maps",
  map_image_filestem = "autosim_map",
}

MAP_BUILDER.use_trajectory_builder_2d = true
MAP_BUILDER.use_trajectory_builder_3d = false

TRAJECTORY_BUILDER_2D.min_range = 0.12
TRAJECTORY_BUILDER_2D.max_range = 20.
TRAJECTORY_BUILDER_2D.missing_data_ray_length = 3.
TRAJECTORY_BUILDER_2D.num_accumulated_range_data = 1
TRAJECTORY_BUILDER_2D.use_imu_data = false
-- Trust wheel odom translation during spin; large CSM linear window causes
-- map→odom jumps when rotating in place (scan match invents translation).
TRAJECTORY_BUILDER_2D.use_online_correlative_scan_matching = true
TRAJECTORY_BUILDER_2D.real_time_correlative_scan_matcher.linear_search_window = 0.05
TRAJECTORY_BUILDER_2D.real_time_correlative_scan_matcher.angular_search_window = math.rad(10.)
TRAJECTORY_BUILDER_2D.real_time_correlative_scan_matcher.translation_delta_cost_weight = 1e1
TRAJECTORY_BUILDER_2D.real_time_correlative_scan_matcher.rotation_delta_cost_weight = 1e-1
TRAJECTORY_BUILDER_2D.ceres_scan_matcher.translation_weight = 20.
TRAJECTORY_BUILDER_2D.ceres_scan_matcher.rotation_weight = 40.
TRAJECTORY_BUILDER_2D.submaps.num_range_data = 60

POSE_GRAPH.optimize_every_n_nodes = 40
POSE_GRAPH.constraint_builder.min_score = 0.55
POSE_GRAPH.optimization_problem.odometry_translation_weight = 1e5
POSE_GRAPH.optimization_problem.odometry_rotation_weight = 1e5

return options
