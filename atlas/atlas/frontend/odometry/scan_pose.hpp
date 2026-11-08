/*
 * Copyright 2026 The Openbot Authors
 */
#ifndef ATLAS_FRONTEND_ODOMETRY_SCAN_POSE_HPP_
#define ATLAS_FRONTEND_ODOMETRY_SCAN_POSE_HPP_

#include <Eigen/Core>
#include <memory>

#include "frontend/inertial/inertial_state.hpp"
#include "map/point_observation.hpp"

namespace atlas {
namespace frontend {
namespace odometry {

using ::atlas::map::PointObservation;
using ::atlas::map::PointObservationList;
using ::atlas::map::SharedPointObservationList;

using ::atlas::frontend::inertial::InertialState;

/**
 * @brief Per-scan pose + optional body cloud (Voxel-SLAM ScanPose).
 *
 * Used by loop buffer / multi-session load; point_observations may be null
 * after handoff.
 */
struct ScanPose {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  InertialState x;
  SharedPointObservationList point_observations;
  Eigen::Matrix<double, 6, 1> v6 = Eigen::Matrix<double, 6, 1>::Zero();

  ScanPose() = default;
  ScanPose(const InertialState& xx,
           SharedPointObservationList point_observation)
      : x(xx), point_observations(std::move(point_observation)) {}

  /** @brief Left-multiply dx onto this pose (loop / PGO update). */
  void Update(const InertialState& dx) {
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

}  // namespace odometry
}  // namespace frontend
}  // namespace atlas

#endif
