-- Planar wheeled robot: raw encoders + IMU -> EKF -> Cartographer.
-- Calibrated base_link -> laser/IMU TF must come from the robot driver/URDF.
include "autosim_2d.lua"

options.odometry_topic = "/odometry/filtered"
options.imu_topic = "/imu"
options.tracking_frame = "base_link"
options.published_frame = "odom"
options.provide_odom_frame = false
options.trajectory_publish_period_sec = 0.2

-- Gyro is already fused into odometry. The planar scan matcher uses that prior.
TRAJECTORY_BUILDER_2D.use_imu_data = false
TRAJECTORY_BUILDER_2D.use_online_correlative_scan_matching = false

return options
