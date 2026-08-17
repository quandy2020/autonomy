from autosim.odometry import WheelOdometry


def test_odometry_tracks_drive_pose_with_zero_noise():
    odom = WheelOdometry(noise_std=0.0, seed=0)
    x, y, yaw = odom.update(gt_x=1.0, gt_y=0.0, gt_yaw=0.1)
    assert (x, y, yaw) == (1.0, 0.0, 0.1)
