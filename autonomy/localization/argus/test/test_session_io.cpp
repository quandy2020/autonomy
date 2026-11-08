/*
 * Copyright 2026 The Openbot Authors
 */

#include <iostream>

#include "argus/system/voxel/serialization/voxel_map_serializer.hpp"

int main() {
    auto specs = argus::vxslam::ParsePreviousMapList(
        "site1: 0.50, site2: 0.45");
    if (specs.size() != 2 || specs[0].name != "site1") {
        std::cerr << "FAIL parse" << std::endl;
        return 1;
    }
    std::vector<argus::vxslam::ImuState> poses(2);
    poses[0].t = 1.0;
    poses[0].p = Eigen::Vector3d(1, 2, 3);
    poses[1].t = 2.0;
    poses[1].p = Eigen::Vector3d(4, 5, 6);
    const char* path = "/tmp/argus_vxslam_traj_test.tum";
    if (!argus::vxslam::SaveTrajectoryTum(path, poses)) {
        std::cerr << "FAIL save" << std::endl;
        return 1;
    }
    std::vector<argus::vxslam::ImuState> loaded;
    if (!argus::vxslam::LoadTrajectoryTum(path, &loaded) ||
        loaded.size() != 2) {
        std::cerr << "FAIL load" << std::endl;
        return 1;
    }
    std::cout << "PASS test_session_io" << std::endl;
    return 0;
}
