/*
 * Copyright 2026 The Openbot Authors
 */

#pragma once

#include <vector>

#include "autonomy/manipulation/dispatch/capability/capability.hpp"
#include "autonomy/manipulation/common/planner_interface.hpp"

namespace autonomy {
namespace manipulation {
namespace dispatch {

/**
 * @brief Capability for check_state_validity / multi-state validation.
 */
class StateValidationCapability : public Capability {
 public:
  std::string Name() const override { return "state_validation"; }
  bool Init(ManipulationServer* server) override;

  /**
   * @brief Validate @p state for collision and optional path constraints.
   * @param[in] state Joint configuration to test.
   * @param[in] constraints Optional constraints (empty = collision only).
   * @return Validity result with error code.
   */
  StateValidationResult CheckState(
      const automsgs::msgs::sensor_msgs::JointState& state,
      const planner::MotionPlanRequest& constraints = {}) const;

  /**
   * @brief Validate each state in @p states.
   * @param[in] states Joint configurations.
   * @param[in] constraints Optional shared constraints.
   * @return Per-state results (same order as @p states).
   */
  std::vector<StateValidationResult> CheckStates(
      const std::vector<automsgs::msgs::sensor_msgs::JointState>& states,
      const planner::MotionPlanRequest& constraints = {}) const;

 private:
  ManipulationServer* server_ = nullptr;
};

}  // namespace dispatch
}  // namespace manipulation
}  // namespace autonomy
