-- Argus Voxel-SLAM options.
-- Loaded by apps/runner or VoxelModule::Configure (YAML bridge pending).

options = {
  module_type = "voxel",
  name = "voxel",

  -- Existing OpenCV FileStorage YAML under config/.
  config_yaml = "config/voxel_slam.yaml",

  odometry = {
    enable_imu = true,
    enable_loop = true,
  },

  local_mapping = {
    window_size = 10,
  },

  global_mapping = {
    use_gtsam = true,
  },
}
