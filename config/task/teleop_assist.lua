-- Copyright 2026 The Openbot Authors
--
-- Teleop MPPI assist (local costmap + path library + MPPI).
-- Point cloud can come from depth_registered, stereo, lidar, etc.

return {
  assist_enabled = false,

  global_frame = "base_link",
  angular_to_dir_gain = 1.0,
  stopped_linear_epsilon = 0.02,
  in_place_angular_scale = 0.5,
  stale_cloud_timeout_sec = 0.5,

  point_cloud = {
    cloud_topic = "/camera/depth/points",
    stale_timeout_sec = 0.5,
  },

  path_library = {
    num_dirs = 9,
    num_lengths = 3,
    max_range = 3.0,
    ds = 0.1,
  },
}
