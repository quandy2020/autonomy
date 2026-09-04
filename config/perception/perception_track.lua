-- Copyright 2026 The Openbot Authors
--
-- Perception preset: YOPO ground-robot human following (no exploration).

AUTONOMY_PERCEPTION = {
  enabled = true,
  enable_rgbd_exploration = false,
  enable_yopo_track = true,
  track_config = "perception/track_yopo.lua",
  cmd_vel_topic = "/cmd_vel",
  odom_topic = "/odom",
  depth_topic = "/camera/depth/image_raw",
  camera_info_topic = "/camera/camera_info",
  camera_frame = "camera_depth_optical_frame",
  map_frame = "map",
  planner_hz = 10.0,
}

return AUTONOMY_PERCEPTION
