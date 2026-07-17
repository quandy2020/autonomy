-- Smoke / dev: fakedata publishes /fake/point_cloud2 (proto sensor_msgs.PointCloud2).
-- Run: TELEOP_ASSIST_CONFIG=task/teleop_assist_smoke.lua ./build/bin/task_teleop_smoke ...

return {
  assist_enabled = true,

  global_frame = "map",
  angular_to_dir_gain = 1.0,
  stopped_linear_epsilon = 0.02,
  in_place_angular_scale = 0.5,
  stale_cloud_timeout_sec = 0.5,

  point_cloud = {
    cloud_topic = "/fake/point_cloud2",
    stale_timeout_sec = 0.5,
  },

  path_library = {
    num_dirs = 9,
    num_lengths = 3,
    max_range = 3.0,
    ds = 0.1,
  },
}
