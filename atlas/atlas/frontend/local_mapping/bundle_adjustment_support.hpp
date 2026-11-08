/*
 * Copyright 2026 The Openbot Authors
 */
#ifndef ATLAS_FRONTEND_LOCAL_MAPPING_BUNDLE_ADJUSTMENT_SUPPORT_HPP_
#define ATLAS_FRONTEND_LOCAL_MAPPING_BUNDLE_ADJUSTMENT_SUPPORT_HPP_

#include <Eigen/Core>
#include <exception>
#include <future>
#include <vector>

#include "common/math/so3.hpp"
#include "common/pose_state.hpp"
#include "frontend/inertial/inertial_state.hpp"

namespace atlas {
namespace frontend {
namespace local_mapping {

using ::atlas::frontend::inertial::InertialState;

inline common::PoseState MakePoseState(const InertialState& inertial_state) {
  return common::PoseState(inertial_state.t, inertial_state.R, inertial_state.p,
                           inertial_state.cov.block<6, 6>(0, 0));
}

inline std::vector<common::PoseState> MakePoseWindow(
    const std::vector<InertialState>& inertial_state_window) {
  std::vector<common::PoseState> pose_window;
  pose_window.reserve(inertial_state_window.size());
  for (const InertialState& inertial_state : inertial_state_window) {
    pose_window.push_back(MakePoseState(inertial_state));
  }
  return pose_window;
}

inline void ApplyPoseWindow(const std::vector<common::PoseState>& pose_window,
                            std::vector<InertialState>* inertial_state_window) {
  if (inertial_state_window == nullptr ||
      pose_window.size() != inertial_state_window->size()) {
    return;
  }
  for (std::size_t pose_index = 0; pose_index < pose_window.size();
       ++pose_index) {
    (*inertial_state_window)[pose_index].R = pose_window[pose_index].rotation;
    (*inertial_state_window)[pose_index].p =
        pose_window[pose_index].translation;
  }
}

inline void WaitForPartitionTasks(std::vector<std::future<void>>& tasks) {
  std::exception_ptr first_exception;
  for (auto& task : tasks) {
    try {
      task.get();
    } catch (...) {
      if (first_exception == nullptr) {
        first_exception = std::current_exception();
      }
    }
  }
  if (first_exception != nullptr) {
    std::rethrow_exception(first_exception);
  }
}

inline Eigen::Matrix3d Exp(const Eigen::Vector3d& vector) {
  return math::Exp(vector);
}

inline Eigen::Matrix3d Exp(const Eigen::Vector3d& vector, double duration) {
  return math::Exp(vector, duration);
}

inline Eigen::Matrix3d hat(const Eigen::Vector3d& vector) {
  return math::SkewSymmetric(vector);
}

#ifndef I33
#define I33 Eigen::Matrix3d::Identity()
#endif

constexpr int DIM = ::atlas::frontend::inertial::kInertialStateDimension;
constexpr int DVEL = 6;

}  // namespace local_mapping
}  // namespace frontend
}  // namespace atlas

#endif
