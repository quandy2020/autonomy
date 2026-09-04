-- Copyright 2026 The Openbot Authors
--
-- Ground-robot YOPO human-following options.

enabled = true
backend_id = "onnx"
-- Leave empty to use heuristic sector-depth fallback (useful for first autosim bring-up).
model_path = ""
allow_heuristic_fallback = true

odom_topic = "/odom"
depth_topic = "/camera/depth/image_raw"
camera_info_topic = "/camera/camera_info"
camera_frame = "camera_depth_optical_frame"
base_frame = "base_link"
cmd_vel_topic = "/cmd_vel"
debug_path_topic = "/tracking/path"
control_hz = 10.0

image_height = 96
image_width = 160
horizon_num = 5
vertical_num = 1
horizon_camera_fov_deg = 90.0
radio_range_m = 3.0
vel_max_mps = 0.5
wz_max_rps = 1.0

objectness_threshold = 0.35
nms_angle_deg = 15.0
follow_distance_m = 1.5
trajectory_mode = "simple"   -- "simple" | "minco" (YOPO-MINCO planar)
minco_piece_duration_s = 1.0
acc_max_mps2 = 1.0
minco_sample_horizon_s = 0.5
