/*
 * Copyright 2026 The Openbot Authors
 */

#include <cmath>
#include <deque>
#include <iostream>

#include "argus/system/voxel/odometry/imu_preintegration.hpp"

namespace {

int Fail(const char* msg) {
    std::cerr << "FAIL: " << msg << std::endl;
    return 1;
}

}  // namespace

int main() {
    using argus::vxslam::ImuPreintegration;
    using argus::vxslam::ImuState;

    argus::vxslam::g_noise_meas.setIdentity();
    argus::vxslam::g_noise_meas *= 1e-4;
    argus::vxslam::g_noise_walk.setIdentity();
    argus::vxslam::g_noise_walk *= 1e-6;

    ImuPreintegration pre;
    std::deque<argus::sensor::ImuData> imus;
    for (int i = 0; i < 20; ++i) {
        argus::sensor::ImuData imu;
        imu.timestamp = 0.01 * i;
        imu.linear_acceleration = Eigen::Vector3d(0, 0, 9.8);
        imu.angular_velocity = Eigen::Vector3d(0.01, 0, 0);
        imus.push_back(imu);
    }
    pre.PushImu(imus);
    if (pre.dtime <= 0) {
        return Fail("dtime");
    }

    ImuState a;
    ImuState b = a;
    b.t = pre.dtime;
    b.p = pre.p_delta;
    b.v = pre.v_delta;
    b.R = pre.R_delta;
    Eigen::MatrixXd jtj;
    Eigen::VectorXd gg;
    const double cost = pre.Evaluate(a, b, jtj, gg, true);
    if (!std::isfinite(cost)) {
        return Fail("cost nan");
    }
    if (jtj.rows() != 30 || jtj.cols() != 30) {
        return Fail("jtj size");
    }
    std::cout << "PASS test_imu_preintegration cost=" << cost << std::endl;
    return 0;
}
