/*
 * Copyright 2026 The Openbot Authors
 */
#ifndef ARGUS_VXSLAM_KDTREE_ODOM_HPP_
#define ARGUS_VXSLAM_KDTREE_ODOM_HPP_

#include <vector>

#include "common/math/kdtree.hpp"
#include "frontend/lidar/imu/imu_state.hpp"
#include "frontend/lidar/odometry/point_var.hpp"
#include "sensor/point_types.hpp"

namespace argus {
namespace vxslam {

/**
 * @brief Point-to-plane EKF using local kd-tree map (Voxel-SLAM
 *        lio_state_estimation_kdtree) — used before voxel map is dense.
 */
class KdTreeOdom {
 public:
  static constexpr int kMatch = 5;

  /** @brief Seed / grow map; returns true when map ready for estimation. */
  bool SeedOrReady(const PointVarList& pvec, const ImuState& x);

  /**
   * @brief Align current scan to kd map; updates `x` in place.
   * @return False if not enough valid associations.
   */
  bool Estimate(PointVarListPtr pvec, ImuState* x);

  void Clear();

  size_t map_size() const { return map_pts_.size(); }

 private:
  std::vector<Eigen::Vector3d> map_pts_;
  math::KdTree kd_;
  sensor::PointCloudXYZI map_cloud_;
};

}  // namespace vxslam
}  // namespace argus

#endif
