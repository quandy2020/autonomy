/*
 * Copyright 2026 The Openbot Authors
 */

#include <cmath>
#include <deque>
#include <iostream>

#include "argus/system/voxel/odometry/imu_ekf.hpp"
#include "argus/system/voxel/odometry/imu_state.hpp"
#include "argus/system/voxel/odometry/point_var.hpp"
#include "argus/mapping/voxel_key.hpp"

namespace {

int Fail(const char* msg) {
    std::cerr << "FAIL: " << msg << std::endl;
    return 1;
}

}  // namespace

int main() {
    using argus::vxslam::ImuState;
    using argus::vxslam::ImuEkf;

    ImuEkf ekf;
    ekf.min_init_num = 5;
    ImuState x;
    argus::sensor::PointCloudXYZI cloud;
    argus::sensor::PointType pt;
    pt.x = 1.f;
    pt.y = 0.f;
    pt.z = 0.f;
    pt.curvature = 0.05f;
    cloud.push_back(pt);

    std::deque<argus::sensor::ImuData> imus;
    for (int i = 0; i < 8; ++i) {
        argus::sensor::ImuData imu;
        imu.timestamp = 0.01 * i;
        imu.linear_acceleration = Eigen::Vector3d(0, 0, 9.8);
        imu.angular_velocity = Eigen::Vector3d::Zero();
        imus.push_back(imu);
    }
    ekf.pcl_beg_time = 0.0;
    ekf.pcl_end_time = 0.07;
    if (ekf.Process(&x, &cloud, &imus) != 0) {
        return Fail("expected init phase return 0");
    }
    // More IMU to finish init
    for (int i = 0; i < 10; ++i) {
        argus::sensor::ImuData imu;
        imu.timestamp = 0.1 + 0.01 * i;
        imu.linear_acceleration = Eigen::Vector3d(0, 0, 9.8);
        imu.angular_velocity = Eigen::Vector3d::Zero();
        imus.push_back(imu);
    }
    ekf.pcl_beg_time = 0.1;
    ekf.pcl_end_time = 0.2;
    const int r = ekf.Process(&x, &cloud, &imus);
    if (!ekf.init_flag) {
        return Fail("init_flag should be true");
    }
    if (r != 1 && r != 0) {
        return Fail("unexpected process return");
    }

    argus::vxslam::PointCluster c;
    c.push(Eigen::Vector3d(1, 0, 0));
    c.push(Eigen::Vector3d(0, 1, 0));
    c.push(Eigen::Vector3d(0, 0, 0));
    if (c.N != 3) {
        return Fail("PointCluster N");
    }
    std::cout << "PASS test_imu_ekf" << std::endl;
    return 0;
}
