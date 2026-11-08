/*
 * Copyright 2026 The Openbot Authors
 */
#ifndef ARGUS_VXSLAM_HIERARCHICAL_BA_HPP_
#define ARGUS_VXSLAM_HIERARCHICAL_BA_HPP_

#include <vector>

#include "argus/optimization/pose_graph.hpp"
#include "argus/sensor/point_types.hpp"
#include "argus/system/voxel/odometry/imu_state.hpp"
#include "argus/system/voxel/global_mapping/pgo_edges.hpp"
#include "argus/system/voxel/local_mapping/sliding_octree.hpp"
#include "argus/system/voxel/local_mapping/octree_gba.hpp"

namespace argus {
namespace vxslam {

struct GbaConfig {
    double voxel_size = 1.0;
    double min_eigen_value = 0.01;
    std::vector<double> eigen_value_array = {1.0, 1.0, 1.0};
    int total_max_iter = 3;
    int thread_num = 4;
};

/**
 * @brief Global BA via OctreeGba planes + LidarBaOptimizer (Voxel-SLAM HBA).
 */
class HierarchicalBa {
public:
    explicit HierarchicalBa(GbaConfig config = {});

    /**
     * @brief Optimize keyframe poses using OctreeGba factors.
     * @return Final residual, or -1 on failure.
     */
    double Optimize(std::vector<ImuState>* poses,
                    const std::vector<sensor::PointCloudXYZI>& clouds);

    /**
     * @brief Bottom-up HBA_add_edge: local GBA then push relative edges
     *        whose Hessian cross-blocks are informative.
     *
     * Filters keyframes whose `mp` is in @p maps. Updates @p xs in place.
     */
    void HbaAddEdge(std::vector<ImuState>* xs,
                    const std::vector<Keyframe*>& smps, PgoEdges* gba_edges,
                    const std::vector<int>& maps, int max_iter,
                    int thread_num);

    /**
     * @brief Merge gba_edges into PoseGraph loop factors for connected maps
     *        (Voxel-SLAM topDownProcess edge injection, no ROS publish).
     */
    static void TopDownApplyEdges(
        backend::PoseGraph* graph, const PgoEdges& gba_edges1,
        const PgoEdges& gba_edges2, const std::vector<int>& ids,
        const std::vector<int>& stepsizes);

private:
    GbaConfig config_;
};

}  // namespace vxslam
}  // namespace argus

#endif
