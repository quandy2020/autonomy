/*
 * Copyright 2026 The Openbot Authors
 */
#include <iostream>

#include "argus/matching/loop_closure/triangle_descriptor/triangle_detector.hpp"
#include "argus/system/voxel/local_mapping/hierarchical_ba.hpp"
#include "argus/system/voxel/odometry/scan_pose.hpp"

namespace {

int Fail(const char* msg) {
    std::cerr << "FAIL: " << msg << std::endl;
    return 1;
}

argus::sensor::PointCloudXYZI MakePlanes(float ox) {
    argus::sensor::PointCloudXYZI cloud;
    for (int plane = 0; plane < 3; ++plane) {
        for (int i = 0; i < 20; ++i) {
            for (int j = 0; j < 20; ++j) {
                argus::sensor::PointType p;
                if (plane == 0) {
                    p.x = ox + 0.05f * i;
                    p.y = 0.05f * j;
                    p.z = 0.f;
                } else if (plane == 1) {
                    p.x = ox + 3.f + 0.05f * i;
                    p.y = 0.05f * j;
                    p.z = 0.f;
                } else {
                    p.x = ox + 1.5f + 0.05f * i;
                    p.y = 3.f + 0.05f * j;
                    p.z = 0.f;
                }
                cloud.push_back(p);
            }
        }
    }
    return cloud;
}

}  // namespace

int main() {
    using argus::loop_closure::KeyFrame;
    using argus::loop_closure::triangle::TriangleDetector;
    using argus::loop_closure::triangle::TriangleConfig;
    using argus::vxslam::HierarchicalBa;
    using argus::vxslam::ImuState;
    using argus::vxslam::PgoEdges;
    using argus::vxslam::ScanPose;

    TriangleConfig cfg;
    cfg.voxel_size = 1.0f;
    cfg.voxel_init_num = 5;
    cfg.skip_near_num = 2;
    cfg.icp_threshold = 0.01f;
    TriangleDetector det(cfg);

    for (int id = 0; id < 8; ++id) {
        KeyFrame kf;
        kf.id = id;
        kf.cloud = MakePlanes(0.f);
        (void)det.Detect(kf);
    }
    KeyFrame q;
    q.id = 100;
    q.cloud = MakePlanes(0.02f);
    const auto cands = det.Detect(q);
    std::cout << "btc SearchLoop cands=" << cands.size()
              << " db=" << det.database_size() << std::endl;

    ScanPose sp;
    sp.x.p = Eigen::Vector3d(1, 2, 3);
    sp.Update(ImuState{});
    if (sp.x.p.norm() < 1e-9) {
        return Fail("ScanPose Update");
    }

    // HbaAddEdge smoke: empty maps → no crash.
    HierarchicalBa gba;
    std::vector<ImuState> xs;
    std::vector<argus::vxslam::Keyframe*> smps;
    PgoEdges edges;
    gba.HbaAddEdge(&xs, smps, &edges, {0}, 1, 1);

    std::cout << "PASS test_triangle_search_loop" << std::endl;
    return 0;
}
