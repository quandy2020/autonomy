/*
 * Copyright 2026 Automanip contributors duyongquan (quandy2020@126.com)
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

/**
 * @file arm_driver.hpp
 * @brief Abstract manipulator backend (hardware or simulated plant).
 *
 * Wire bodies are automsgs: PoseStamped targets, JointCommand, JointState.
 * The manager owns Autolink. The driver owns the plant.
 */

#ifndef AUTOMANIP_ARM_ARM_DRIVER_HPP_
#define AUTOMANIP_ARM_ARM_DRIVER_HPP_

#include <string>

#include "automanip/arm/mode.hpp"
#include "autolink/common/macros.hpp"

#include <automsgs/msgs/control_msgs/joint_command.pb.h>
#include <automsgs/msgs/geometry_msgs/pose_stamped.pb.h>
#include <automsgs/msgs/sensor_msgs/joint_state.pb.h>

namespace automanip {
namespace arm {

using PoseCommand = automsgs::msgs::geometry_msgs::PoseStamped;
using JointCommand = automsgs::msgs::control_msgs::JointCommand;
using JointState = automsgs::msgs::sensor_msgs::JointState;

/**
 * @brief Vendor or simulated plant: targets in, joint state out.
 */
class ArmDriver {
 public:
  AUTOLINK_SHARED_PTR_DEFINITIONS(ArmDriver)
  DISALLOW_COPY_AND_ASSIGN(ArmDriver)

  virtual ~ArmDriver() = default;

  virtual const std::string& GetArmId() const = 0;
  virtual bool Start() = 0;
  virtual void Stop() = 0;
  virtual bool IsRunning() const = 0;

  /** @brief Advance the plant by @p dt seconds. */
  virtual bool Step(double dt) = 0;

  virtual bool ApplyPoseTarget(const PoseCommand& pose) = 0;
  virtual bool ApplyJointCommand(const JointCommand& command) = 0;
  virtual bool ApplyMode(const std::string& mode) = 0;
  virtual ArmMode GetMode() const = 0;

  virtual bool ReadJointState(JointState* state) const = 0;
  virtual bool ReadEndEffectorPose(PoseCommand* pose) const = 0;
  virtual bool TriggerEmergencyStop() = 0;

 protected:
  ArmDriver() = default;
};

}  // namespace arm
}  // namespace automanip

#endif  // AUTOMANIP_ARM_ARM_DRIVER_HPP_
