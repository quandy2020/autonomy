/*
 * Copyright 2026 The Openbot Authors
 */
#ifndef ATLAS_MAP_HIERARCHICAL_BUNDLE_ADJUSTMENT_HPP_
#define ATLAS_MAP_HIERARCHICAL_BUNDLE_ADJUSTMENT_HPP_

#include <memory>
#include <vector>

#include "common/pose_state.hpp"
#include "common/session_pose_constraint.hpp"
#include "common/thread_pool.hpp"
#include "sensor/point_types.hpp"

namespace atlas {
namespace map {

using ::atlas::common::PoseState;

struct GlobalBundleAdjustmentConfiguration {
  double voxel_size = 1.0;
  double minimum_eigenvalue = 0.01;
  std::vector<double> eigenvalue_thresholds = {1.0, 1.0, 1.0};
  int maximum_octree_layer = 2;
  int maximum_iteration_count = 3;
  int worker_thread_count = 4;
};

/** @brief Read-only keyframe input used by one bundle-adjustment operation. */
struct BundleAdjustmentKeyframe {
  int keyframe_id = -1;
  int session_id = -1;
  std::shared_ptr<const sensor::PointCloudXYZI> cloud;
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
  double Optimize(std::vector<PoseState>* poses,
                  const std::vector<sensor::PointCloudXYZI>& clouds);
  /** @brief Optimize immutable shared snapshots without copying point data. */
  double Optimize(
      std::vector<PoseState>* poses,
      const std::vector<std::shared_ptr<const sensor::PointCloudXYZI>>& clouds);

  /**
   * @brief Optimize local windows and emit informative relative pose edges.
   *
   * Filters keyframes whose `session_id` is in @p session_ids. Updates @p poses
   * in place.
   */
  void AddEdgesFromHierarchicalBundleAdjustment(
      std::vector<PoseState>* poses,
      const std::vector<BundleAdjustmentKeyframe>& keyframes,
      common::SessionPoseConstraints* pose_constraints,
      const std::vector<int>& session_ids, int maximum_iteration_count,
      int worker_thread_count,
      const GlobalBundleAdjustmentConfiguration& configuration,
      const GlobalBundleAdjustmentConfiguration* final_iteration_configuration =
          nullptr);

 private:
  GlobalBundleAdjustmentConfiguration configuration_;
  common::ThreadPool thread_pool_;
};

}  // namespace map
}  // namespace atlas

#endif
