/*
 * Copyright 2026 The Openbot Authors
 */
#ifndef ATLAS_COMMON_POSE_STATE_HPP_
#define ATLAS_COMMON_POSE_STATE_HPP_

#include <Eigen/Core>

namespace atlas {
namespace common {

/** @brief Pose and its local six-dimensional covariance. */
struct PoseState {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  using Covariance = Eigen::Matrix<double, 6, 6>;

  double timestamp = 0.0;
  Eigen::Matrix3d rotation = Eigen::Matrix3d::Identity();
  Eigen::Vector3d translation = Eigen::Vector3d::Zero();
  Covariance covariance = Covariance::Identity() * 1e-4;

  PoseState() = default;

  PoseState(double pose_timestamp, const Eigen::Matrix3d& pose_rotation,
            const Eigen::Vector3d& pose_translation,
            const Covariance& pose_covariance)
      : timestamp(pose_timestamp),
        rotation(pose_rotation),
        translation(pose_translation),
        covariance(pose_covariance) {}
};

}  // namespace common
}  // namespace atlas

#endif  // ATLAS_COMMON_POSE_STATE_HPP_
