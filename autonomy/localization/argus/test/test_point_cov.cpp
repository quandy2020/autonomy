/*
 * Copyright 2026 The Openbot Authors
 */
#include <iostream>

#include "argus/system/voxel/odometry/point_cov.hpp"

int main() {
    Eigen::Vector3d pb(1, 0, 2);
    Eigen::Matrix3d var;
    argus::vxslam::CalcBodyVar(pb, 0.05f, 0.2f, &var);
    if (var.norm() < 1e-9) {
        std::cerr << "FAIL var" << std::endl;
        return 1;
    }
    argus::sensor::PointCloudXYZI cloud;
    for (int i = 0; i < 5; ++i) {
        argus::sensor::PointType p;
        p.x = 0.1f * i;
        p.y = 0;
        p.z = 1;
        cloud.push_back(p);
    }
    argus::vxslam::ImuState ext;
    argus::vxslam::PointVarList pvec;
    argus::vxslam::InitPointVars(ext, cloud, 0.05, 0.2, &pvec);
    if (pvec.size() != 5) {
        std::cerr << "FAIL init size" << std::endl;
        return 1;
    }
    argus::vxslam::VectorList<3> pwld;
    argus::vxslam::UpdateWorldVars(ext, &pvec, &pwld);
    if (pwld.size() != 5) {
        std::cerr << "FAIL pwld" << std::endl;
        return 1;
    }
    std::cout << "PASS test_point_cov" << std::endl;
    return 0;
}
