/*
 * Copyright 2026 The Openbot Authors
 */
#ifndef ATLAS_VXSLAM_SCAN_POSE_HPP_
#define ATLAS_VXSLAM_SCAN_POSE_HPP_

#include <Eigen/Core>
#include <memory>

#include "frontend/lidar/imu/imu_state.hpp"
#include "frontend/lidar/odometry/point_var.hpp"

namespace atlas {
namespace vxslam {

/**
 * @brief Per-scan pose + optional body cloud (Voxel-SLAM ScanPose).
 *
 * Used by loop buffer / multi-session load; pvec may be null after handoff.
 */
struct ScanPose {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  ImuState x;
  PointVarListPtr pvec;
  Eigen::Matrix<double, 6, 1> v6 = Eigen::Matrix<double, 6, 1>::Zero();

  ScanPose() = default;
  ScanPose(const ImuState& xx, PointVarListPtr pv)
      : x(xx), pvec(std::move(pv)) {}

  /** @brief Left-multiply dx onto this pose (loop / PGO update). */
  void Update(const ImuState& dx) {
    x.v = dx.R * x.v;
    x.p = dx.R * x.p + dx.p;
    x.R = dx.R * x.R;
  }

  /** @brief Set absolute pose; rotate velocity by ΔR. */
  void SetState(const Eigen::Matrix3d& R, const Eigen::Vector3d& p) {
    const Eigen::Matrix3d dR = R * x.R.transpose();
    x.R = R;
    x.p = p;
    x.v = dR * x.v;
  }
};

using ScanPosePtr = std::shared_ptr<ScanPose>;

}  // namespace vxslam
}  // namespace atlas

#endif
