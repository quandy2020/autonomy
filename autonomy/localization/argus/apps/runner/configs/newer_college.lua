-- Argus offline runner options (Newer College).
-- Usage: argus_runner --config apps/runner/configs/newer_college.lua --data <seq_dir>

options = {
  dataset = "newer_college",
  data_path = "",
  config_yaml = "config/livo_esikf.yaml",
  camera = "cam0",
  max_frames = 0,
}
