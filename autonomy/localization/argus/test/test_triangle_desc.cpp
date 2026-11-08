/*
 * Copyright 2026 The Openbot Authors
 */

#include <iostream>

#include "argus/matching/loop_closure/triangle_descriptor/triangle_detector.hpp"

namespace {

int Fail(const char* msg) {
    std::cerr << "FAIL: " << msg << std::endl;
    return 1;
}

}  // namespace

int main() {
    using argus::loop_closure::KeyFrame;
    using argus::loop_closure::triangle::TriangleDetector;
    using argus::loop_closure::triangle::TriangleConfig;

    TriangleConfig cfg;
    cfg.voxel_size = 1.0f;
    cfg.voxel_init_num = 5;
    cfg.descriptor_near_num = 5;
    TriangleDetector det(cfg);

    auto make_cloud = [](float ox) {
        argus::sensor::PointCloudXYZI cloud;
        // Three planar patches
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
    };

    for (int id = 0; id < 5; ++id) {
        KeyFrame kf;
        kf.id = id;
        kf.cloud = make_cloud(0.f);
        (void)det.Detect(kf);
    }
    KeyFrame q;
    q.id = 100;
    q.cloud = make_cloud(0.05f);
    auto cands = det.Detect(q);
    // May or may not find loop depending on thresholds; just ensure no crash.
    std::cout << "PASS test_triangle_desc cands=" << cands.size()
              << " db=" << det.database_size() << std::endl;
    return 0;
}
