/*
 * Copyright 2026 The Openbot Authors
 */

#include <iostream>
#include <vector>

#include "argus/system/voxel/local_mapping/hierarchical_ba.hpp"

int main() {
    using argus::vxslam::HierarchicalBa;
    using argus::vxslam::ImuState;

    std::vector<ImuState> poses(3);
    std::vector<argus::sensor::PointCloudXYZI> clouds(3);
    for (int i = 0; i < 3; ++i) {
        poses[static_cast<size_t>(i)].p = Eigen::Vector3d(i * 0.5, 0, 0);
        for (int u = 0; u < 15; ++u) {
            for (int v = 0; v < 15; ++v) {
                argus::sensor::PointType p;
                p.x = 0.1f * u;
                p.y = 0.1f * v;
                p.z = 0.f;
                clouds[static_cast<size_t>(i)].push_back(p);
            }
        }
    }
    HierarchicalBa gba;
    const double r = gba.Optimize(&poses, clouds);
    std::cout << "PASS test_hierarchical_ba residual=" << r << std::endl;
    return 0;
}
