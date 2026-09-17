/*
 * Copyright 2026 The Openbot Authors
 */

#pragma once

#include "autonomy/manipulation/dispatch/capability/capability.hpp"
#include "autonomy/manipulation/model/error_codes.hpp"
#include "autonomy/manipulation/common/planner_interface.hpp"

namespace autonomy {
namespace manipulation {
namespace dispatch {

/**
 * @brief Capability that validates an arbitrary trajectory against the scene.
 */
class ValidateTrajectoryCapability : public Capability {
 public:
  std::string Name() const override { return "validate_trajectory"; }
  bool Init(ManipulationServer* server) override;

  /**
   * @brief Check path validity (collision / occupancy) for @p trajectory.
   * @param[in] trajectory Waypoints to validate.
   * @param[in] constraints Optional joint/pose path constraints.
   * @return ErrorCode::SUCCESS if valid.
   */
  ErrorCode Validate(
      const automsgs::msgs::trajectory_msgs::JointTrajectory& trajectory,
      const planner::MotionPlanRequest& constraints = {}) const;

 private:
  ManipulationServer* server_ = nullptr;
};

}  // namespace dispatch
}  // namespace manipulation
}  // namespace autonomy
