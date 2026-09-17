/*
 * Copyright 2026 The Openbot Authors
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#pragma once

#include <memory>
#include <string>

#include <automsgs/msgs/trajectory_msgs/joint_trajectory.pb.h>

#include "autonomy/common/macros.hpp"

namespace autonomy {
namespace manipulation {
namespace common {

/**
 * @class ControllerInterface
 * @brief Abstract joint-trajectory controller plugin base.
 *
 * Concrete backends are registered via the manipulation plugin hub and selected
 * by TrajectoryExecutionManager. Implementations typically publish or apply
 * trajectory_msgs/JointTrajectory to hardware or a simulation bridge.
 */
class ControllerInterface
{
public:
  /**
   * @brief Define ControllerInterface::SharedPtr type
   */
  AUTONOMY_SMART_PTR_DEFINITIONS(ControllerInterface)

  /**
   * @brief Destructor for ControllerInterface
   */
  virtual ~ControllerInterface() = default;

  /**
   * @brief Initialize the controller with a config / hardware identifier.
   * @param controller_id Plugin alias or hardware controller name from
   *        configuration.
   * @return true if initialization succeeds and the controller is ready to
   *         accept FollowJointTrajectory.
   */
  virtual bool Init(const std::string& controller_id) = 0;

  /**
   * @brief Follow a joint-space reference trajectory until completion or cancel.
   *
   * Blocking call: returns only after the trajectory finishes, fails, or
   * Cancel is invoked. Waypoints carry joint positions and optional
   * velocities / efforts with time_from_start stamps.
   *
   * @param joint_trajectory Reference trajectory to track.
   * @return true if following completed successfully; false on empty input,
   *         hardware failure, or cancellation.
   */
  virtual bool FollowJointTrajectory(
      const automsgs::msgs::trajectory_msgs::JointTrajectory&
          joint_trajectory) = 0;

  /**
   * @brief Request cancellation of an in-flight FollowJointTrajectory.
   *
   * Safe to call when idle; an active follow should return false promptly.
   */
  virtual void Cancel() = 0;

  /**
   * @brief Whether the controller currently holds an active follow goal.
   * @return true while FollowJointTrajectory is in progress.
   */
  virtual bool IsActive() const = 0;

protected:
  /**
   * @brief Default constructor for plugin registration only.
   */
  ControllerInterface() = default;
};

}  // namespace common

namespace execution {
using ControllerInterface = common::ControllerInterface;
}  // namespace execution

}  // namespace manipulation
}  // namespace autonomy
