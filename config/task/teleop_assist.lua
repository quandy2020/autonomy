-- Copyright 2026 The Openbot Authors
--
-- Teleop MPPI assist: rolling local costmap + CMU/FALCO polar spline paths + MPPI.
-- Path library follows CMU local_planner path_generator.m (JFR 2020 FALCO).
-- All planning / visualization is in base_link (no map / global frame).

return {
  assist_enabled = true,
  publish_path_viz = true,
  path_viz_rate_hz = 15.0,

  angular_to_dir_gain = 1.0,
  stopped_linear_epsilon = 0.02,
  in_place_angular_scale = 0.5,
  -- Forward blocked: zero linear, safe in-place turn toward joystick heading.
  blocked_turn_enabled = true,
  blocked_turn_probe_length = 0.35,
  blocked_turn_max_angular = 0.8,
  stale_cloud_timeout_sec = 0.5,

  -- Local costmap input: RGB-D point cloud only (no laser).
  use_laser_scan = false,

  point_cloud = {
    cloud_topic = "/camera/depth/points",
    depth_topic = "/camera/depth/image_raw",
    camera_info_topic = "/camera/camera_info",
    depth_decimation = 4,
    min_depth_m = 0.15,
    max_depth_m = 3.5,
    stale_timeout_sec = 0.5,
  },

  -- Rolling local costmap in base_link (meters); RGB-D marks obstacles only.
  local_costmap = {
    width = 7,
    height = 7,
    resolution = 0.05,
    update_frequency = 20.0,
    robot_radius = 0.22,
  },

  path_library = {
    use_polar_spline = true,
    -- CMU path_generator.m defaults
    segment_length = 1.0,
    max_heading_deg = 27.0,
    hierarchy_scale = 0.65,
    path_range = 3.0,
    sample_ds = 0.05,
    library_knot_ds = 0.01,
    -- FALCO sensor_range: 0 = auto from point_cloud.max_depth_m
    sensor_range = 0.0,
    use_group_start_path = false,
    -- CMU local_planner / FALCO selection
    min_path_range = 1.0,
    path_range_step = 0.5,
    look_ahead_distance = 0.5,
    use_group_selection = true,
    point_per_path_thr = 2,
    dir_weight = 0.02,
    dir_threshold_deg = 90.0,
    path_range_by_speed = true,
    max_linear_speed = 0.5,
    use_rot_dir_search = true,
    num_rot_dirs = 36,
    def_path_scale = 1.25,
    min_path_scale = 0.75,
    path_scale_step = 0.25,
    path_scale_by_speed = true,
    dir_to_vehicle = false,
    check_rot_obstacle = true,
    rot_obstacle_vehicle_length = 0.6,
    rot_obstacle_vehicle_width = 0.4,
    two_way_drive = true,
    -- Costmap traversability (FALCO-style) + teleop smoothness terms
    traversability_weight = 0.25,
    clearance_weight = 0.25,
    smoothness_weight = 0.15,
    efficiency_weight = 0.1,
    velocity_continuity_weight = 0.2,
    temporal_weight = 0.25,
    normalize_group_scores = true,
    plot_path_set = true,
    -- RGB-D FoV gate for free_path_markers (autosim hfov_deg = 90)
    rgbd_hfov_deg = 90.0,
    rgbd_camera_offset_x = 0.03,
    rgbd_camera_offset_y = 0.0,
    free_paths_in_base_link = true,
    -- FALCO paper Fig.3/4: symmetric 7-group fan, clip at obstacles
    free_paths_plot_library_fan = true,
    free_paths_max_markers = 343,
    free_paths_filter_collisions = true,
    -- Legacy arc library (use_polar_spline=false):
    num_dirs = 9,
    num_lengths = 3,
    max_range = 3.0,
    ds = 0.1,
  },
}
