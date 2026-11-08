-- Argus offline runner options (TUM RGB-D; trajectory eval / future playback).
-- Usage: argus_runner --config apps/runner/configs/tum_rgbd.lua --data <seq_dir>

options = {
  dataset = "tum_rgbd",
  data_path = "",
  config_yaml = "config/livo_esikf.yaml",
  max_frames = 0,
}
