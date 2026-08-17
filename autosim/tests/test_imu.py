from autosim.imu import ImuEstimator


def test_imu_finite_difference():
    imu = ImuEstimator()
    imu.reset(yaw=0.0, t=0.0, v=1.0)
    gyro_z, ax, ay = imu.update(yaw=0.1, v=1.0, t=0.1)
    assert abs(gyro_z - 1.0) < 1e-6
    assert abs(ax) < 1e-6  # constant speed
