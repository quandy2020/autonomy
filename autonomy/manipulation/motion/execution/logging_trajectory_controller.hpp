/*
 * Copyright 2026 The Openbot Authors
 *
 * Logs trajectory execution (hardware interface TBD).
 */

#pragma once

#include <string>

#include "autonomy/manipulation/common/controller_interface.hpp"

namespace autonomy {
namespace manipulation {
namespace execution {

/**
 * @brief No-op / log-only controller for tests and bring-up without hardware.
 */
class LoggingTrajectoryController : public ControllerInterface {
 public:
  /**
   * @brief Store the controller id and mark the instance ready.
   * @param[in] controller_id Identifier written into logs.
   * @return Always true.
   */
  bool Init(const std::string& controller_id) override;

  /**
   * @brief Log the trajectory and mark the controller inactive when finished.
   * @param[in] trajectory Joint waypoints (not sent to hardware).
   * @return true unless Cancel was requested mid-run.
   */
  bool FollowJointTrajectory(const automsgs::msgs::trajectory_msgs::JointTrajectory&
                   joint_trajectory) override;

  /** @brief Signal Cancel so an in-flight FollowJointTrajectory returns false. */
  void Cancel() override;

  /** @brief Whether Execute is currently considered active. */
  bool IsActive() const override;

 private:
  std::string controller_id_;
  bool active_ = false;
};

}  // namespace execution
}  // namespace manipulation
}  // namespace autonomy
