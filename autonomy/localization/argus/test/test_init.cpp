/*
 * Copyright 2026 The Openbot Authors
 */

#include <deque>
#include <iostream>

#include "argus/system/voxel/initialization/initialization.hpp"

int main() {
    std::deque<argus::sensor::ImuData> imus;
    for (int i = 0; i < 20; ++i) {
        argus::sensor::ImuData imu;
        imu.timestamp = 0.01 * i;
        imu.linear_acceleration = Eigen::Vector3d(0.1, -0.2, 9.7);
        imus.push_back(imu);
    }
    argus::vxslam::ImuState x;
    if (!argus::vxslam::AlignGravity(imus, &x)) {
        std::cerr << "FAIL AlignGravity" << std::endl;
        return 1;
    }
    if (!argus::vxslam::MotionInitReady(5, 5)) {
        std::cerr << "FAIL MotionInitReady" << std::endl;
        return 1;
    }
    std::cout << "PASS test_init g=" << x.g.transpose() << std::endl;
    return 0;
}
