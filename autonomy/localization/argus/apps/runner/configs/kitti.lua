-- Argus offline runner options (KITTI odometry subset).
-- Usage: argus_runner --config apps/runner/configs/kitti.lua --data <subset_dir>

options = {
  dataset = "kitti",
  -- Path overridden by --data on the CLI.
  data_path = "",
  -- LIVO / LO YAML under config/.
  config_yaml = "config/livo_esikf.yaml",
  max_frames = 0,
  filter_only = false,
}
