/*
 * Copyright 2026 The Openbot Authors
 */
#include <iostream>

#include "argus/system/voxel/global_mapping/pgo_edges.hpp"

int main() {
    argus::vxslam::PgoEdges edges;
    Eigen::Matrix3d R = Eigen::Matrix3d::Identity();
    Eigen::Vector3d t(1, 0, 0);
    Eigen::Matrix<double, 6, 1> v6;
    v6.setConstant(1e-6);
    edges.Push(0, 1, 0, 5, R, t, v6);
    if (edges.edges.size() != 1 || edges.edges[0].ids1[0] != 0) {
        std::cerr << "FAIL push" << std::endl;
        return 1;
    }

    std::vector<argus::vxslam::ImuState> poses(3);
    for (int i = 0; i < 3; ++i) {
        poses[static_cast<size_t>(i)].p = Eigen::Vector3d(i, 0, 0);
    }
    argus::backend::PoseGraph graph;
    argus::vxslam::BuildPoseGraph(&graph, poses, {});
    auto opt = graph.Optimize(5);
    if (!opt.ok) {
        std::cerr << "FAIL optimize" << std::endl;
        return 1;
    }
    std::cout << "PASS test_pgo_edges" << std::endl;
    return 0;
}
