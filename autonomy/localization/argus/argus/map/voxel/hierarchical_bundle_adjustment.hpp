/*
 * Copyright 2026 The Openbot Authors
 */
#ifndef ARGUS_MAP_VOXEL_HIERARCHICAL_BUNDLE_ADJUSTMENT_HPP_
#define ARGUS_MAP_VOXEL_HIERARCHICAL_BUNDLE_ADJUSTMENT_HPP_

#include <memory>
#include <vector>

#include "backend/global_mapping/pgo_edges.hpp"
#include "common/thread_pool.hpp"
#include "frontend/lidar/imu/imu_state.hpp"
#include "map/voxel/keyframe.hpp"
#include "optimization/voxel/pose_graph.hpp"
#include "sensor/point_types.hpp"

namespace argus {
namespace vxslam {

struct GlobalBundleAdjustmentConfiguration {
  double voxel_size = 1.0;
  double minimum_eigenvalue = 0.01;
  std::vector<double> eigenvalue_thresholds = {1.0, 1.0, 1.0};
  int maximum_octree_layer = 2;
  int maximum_iteration_count = 3;
  int worker_thread_count = 4;
};

/**
 * @brief Hierarchical bundle adjustment using voxel plane factors.
 */
class HierarchicalBundleAdjustment {
 public:
  explicit HierarchicalBundleAdjustment(
      GlobalBundleAdjustmentConfiguration configuration = {});

  /**
   * @brief Optimize keyframe poses using GlobalBundleAdjustmentOctree factors.
   * @return Final residual, or -1 on failure.
   */
  double Optimize(std::vector<ImuState>* poses,
                  const std::vector<sensor::PointCloudXYZI>& clouds);
  /** @brief Optimize immutable shared snapshots without copying point data. */
  double Optimize(
      std::vector<ImuState>* poses,
      const std::vector<std::shared_ptr<const sensor::PointCloudXYZI>>& clouds);

  /**
   * @brief Optimize local windows and emit informative relative pose edges.
   *
   * Filters keyframes whose `mp` is in @p session_ids. Updates @p poses in
   * place.
   */
  void AddEdgesFromHierarchicalBundleAdjustment(
      std::vector<ImuState>* poses, const std::vector<Keyframe*>& keyframes,
      PgoEdges* bundle_adjustment_edges, const std::vector<int>& session_ids,
      int maximum_iteration_count, int worker_thread_count);

  /**
   * @brief Apply local and global constraints to connected sessions.
   *        (Voxel-SLAM topDownProcess edge injection, no ROS publish).
   */
  static void TopDownApplyEdges(backend::PoseGraph* graph,
                                const PgoEdges& local_bundle_adjustment_edges,
                                const PgoEdges& global_bundle_adjustment_edges,
                                const std::vector<int>& session_ids,
                                const std::vector<int>& session_offsets);

 private:
  GlobalBundleAdjustmentConfiguration configuration_;
  common::ThreadPool thread_pool_;
};

}  // namespace vxslam
}  // namespace argus

#endif
