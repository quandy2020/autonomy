/*
 * Copyright 2026 The Openbot Authors
 *
 * Mutable robot configuration (joint positions / group subsets).
 */

#pragma once

#include <string>
#include <vector>

#include "autonomy/common/macros.hpp"

#include <automsgs/msgs/sensor_msgs/joint_state.pb.h>

namespace autonomy {
namespace manipulation {
namespace model {

/**
 * @class RobotState
 * @brief Mutable robot configuration (joint positions / group subsets).
 */
class RobotState {
 public:
  AUTONOMY_SMART_PTR_DEFINITIONS(RobotState)

  virtual ~RobotState() = default;

  /**
   * @brief Replace the full joint state.
   * @param[in] state Named positions (and optional velocities).
   */
  virtual void SetJointState(
      const automsgs::msgs::sensor_msgs::JointState& state) = 0;

  /** @brief Current full joint state. */
  virtual automsgs::msgs::sensor_msgs::JointState GetJointState() const = 0;

  /**
   * @brief Set positions for joints in a planning group.
   * @param[in] group Planning group name.
   * @param[in] positions Values ordered as @ref RobotModel::GetJointNames.
   */
  virtual void SetJointGroupPositions(
      const std::string& group, const std::vector<double>& positions) = 0;

  /**
   * @brief Read positions for joints in a planning group.
   * @param[in] group Planning group name.
   * @return Positions ordered as @ref RobotModel::GetJointNames.
   */
  virtual std::vector<double> GetJointGroupPositions(
      const std::string& group) const = 0;

  /**
   * @brief Linearly interpolate this state toward @p to.
   * @param[in] to Target state.
   * @param[in] t Interpolation factor in [0, 1].
   * @param[out] result Interpolated state (must be non-null).
   */
  virtual void Interpolate(const RobotState& to, double t,
                           RobotState* result) const = 0;

 protected:
  RobotState() = default;
};

}  // namespace model
}  // namespace manipulation
}  // namespace autonomy
