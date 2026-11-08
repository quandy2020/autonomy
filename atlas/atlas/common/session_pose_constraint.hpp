/*
 * Copyright 2026 The Openbot Authors
 */
#ifndef ATLAS_COMMON_SESSION_POSE_CONSTRAINT_HPP_
#define ATLAS_COMMON_SESSION_POSE_CONSTRAINT_HPP_

#include <Eigen/Core>
#include <vector>

#include "common/transform/rigid_transform.hpp"

namespace atlas {
namespace common {

/** @brief Relative pose measurement between keyframes in two sessions. */
struct SessionPoseConstraint {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  int from_session_id = -1;
  int to_session_id = -1;
  int from_keyframe_id = -1;
  int to_keyframe_id = -1;
  transform::Rigid3d relative_pose = transform::Rigid3d::Identity();
  Eigen::Matrix<double, 6, 1> diagonal_variance =
      Eigen::Matrix<double, 6, 1>::Zero();
};

using SessionPoseConstraints =
    std::vector<SessionPoseConstraint,
                Eigen::aligned_allocator<SessionPoseConstraint>>;

}  // namespace common
}  // namespace atlas

#endif  // ATLAS_COMMON_SESSION_POSE_CONSTRAINT_HPP_
