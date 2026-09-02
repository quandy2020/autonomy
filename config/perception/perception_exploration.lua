-- Copyright 2025 The Openbot Authors(duyongquan)
--
-- Exploration-enabled perception preset (used by exploration_autonomy.lua).

AUTONOMY_PERCEPTION = {
  enabled = true,
  enable_rgbd_exploration = true,
  exploration_config = "perception/exploration_rgbd_tare.lua",
  odom_topic = "/odom",
  depth_topic = "/camera/depth/image_raw",
  camera_info_topic = "/camera/depth/camera_info",
  camera_frame = "camera_depth_optical_frame",
  map_frame = "map",
  planner_hz = 2.0,
  path_topic = "/exploration/path",
  waypoint_topic = "/exploration/waypoint",
  map_topic = "/exploration/map",
  global_path_topic = "/exploration/global_path",
  local_path_topic = "/exploration/local_path",
  exploration_finished_topic = "/exploration/finished",
  exploration_progress_topic = "/exploration/progress",
  navigation_boundary_topic = "/exploration/navigation_boundary",
  terrain_map_topic = "/terrain_map",
  enable_exploration_nav_bridge = true,
  explorer_backend = "rgbd_tare",
  point_cloud_topic = "/velodyne_points",
}

return AUTONOMY_PERCEPTION
