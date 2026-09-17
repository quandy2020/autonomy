/*
 * Copyright 2026 The Openbot Authors
 *
 * Trajectory controller interface (MoveIt controller_manager analogue).
 */

#pragma once

#include <string>

#include "autonomy/manipulation/model/robot_model.hpp"

namespace autonomy {
namespace manipulation {
namespace execution {

/**
 * @brief Abstract trajectory controller used by TrajectoryExecutionManager.
 */
class ControllerManager {
 public:
  virtual ~ControllerManager() = default;

  /**
   * @brief Initialize the controller with a config / hardware id.
   * @param[in] controller_id Plugin or hardware controller identifier.
   * @return true on successful initialization.
   */
  virtual bool Init(const std::string& controller_id) = 0;

  /**
   * @brief Execute a joint-space trajectory (blocking until done or cancelled).
   * @param[in] trajectory Waypoints to follow.
   * @return true if execution completed successfully.
   */
  virtual bool Execute(const core::RobotTrajectory& trajectory) = 0;

  /** @brief Request cancellation of an in-flight Execute. */
  virtual void Cancel() = 0;

  /** @brief Whether the controller currently holds an active goal. */
  virtual bool IsActive() const = 0;
};

}  // namespace execution
}  // namespace manipulation
}  // namespace autonomy
