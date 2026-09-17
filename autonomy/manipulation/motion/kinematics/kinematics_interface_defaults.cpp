/*
 * Copyright 2026 The Openbot Authors
 */

#include "autonomy/manipulation/common/kinematics_interface.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <random>

namespace autonomy {
namespace manipulation {
namespace common {

ErrorCode KinematicsInterface::SearchPositionIK(const automsgs::msgs::geometry_msgs::Pose& tip_pose,
                                           const automsgs::msgs::sensor_msgs::JointState& seed,
                                           const InverseKinematicsOptions& options,
                                           automsgs::msgs::sensor_msgs::JointState* solution) const {
  if (!solution) {
    return ErrorCode::NO_INVERSE_KINEMATICS_SOLUTION;
  }
  const auto t0 = std::chrono::steady_clock::now();
  const double timeout_s = options.timeout() > 0.0 ? options.timeout() : 0.05;
  const int max_attempts = std::max(1, options.max_attempts());

  ErrorCode last = GetPositionIK(tip_pose, seed, options, solution);
  if (last == ErrorCode::SUCCESS) {
    return last;
  }

  std::mt19937 rng(42);
  std::uniform_real_distribution<double> uni(-0.2, 0.2);
  for (int a = 1; a < max_attempts; ++a) {
    const auto elapsed = std::chrono::duration<double>(
                             std::chrono::steady_clock::now() - t0)
                             .count();
    if (elapsed >= timeout_s) {
      break;
    }
    automsgs::msgs::sensor_msgs::JointState perturbed = seed;
    for (int i = 0; i < perturbed.position_size(); ++i) {
      perturbed.set_position(i, perturbed.position(i) + uni(rng));
    }
    last = GetPositionIK(tip_pose, perturbed, options, solution);
    if (last == ErrorCode::SUCCESS) {
      return last;
    }
  }
  return last == ErrorCode::SUCCESS ? ErrorCode::SUCCESS
                                     : ErrorCode::NO_INVERSE_KINEMATICS_SOLUTION;
}

}  // namespace common
}  // namespace manipulation
}  // namespace autonomy
