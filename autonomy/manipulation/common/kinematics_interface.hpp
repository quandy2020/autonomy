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

#include <automsgs/msgs/geometry_msgs/pose.pb.h>
#include <automsgs/msgs/sensor_msgs/joint_state.pb.h>

#include "autonomy/common/macros.hpp"
#include "autonomy/manipulation/model/error_codes.hpp"
#include "autonomy/manipulation/proto/inverse_kinematics_options.pb.h"

namespace autonomy {
namespace manipulation {
namespace common {

/**
 * @brief Inverse kinematics solver options (proto wire type).
 */
using InverseKinematicsOptions = ::autonomy::manipulation::proto::InverseKinematicsOptions;

/**
 * @class KinematicsInterface
 * @brief Abstract FK/IK solver plugin for a planning group chain.
 *
 * Concrete backends (KDL, TRAC-IK, IKFast, …) are registered via the
 * manipulation plugin hub and bound to a group / base / tip frame on Init.
 */
class KinematicsInterface
{
public:
  /**
   * @brief Define KinematicsInterface::SharedPtr type
   */
  AUTONOMY_SMART_PTR_DEFINITIONS(KinematicsInterface)

  /**
   * @brief Destructor for KinematicsInterface
   */
  virtual ~KinematicsInterface() = default;

  /**
   * @brief Bind the solver to a planning group and chain frames.
   * @param group Planning group name.
   * @param base_frame Chain base frame id.
   * @param tip_frame Chain tip / EE frame id.
   * @return true on successful initialization.
   */
  virtual bool Init(const std::string& group, const std::string& base_frame,
                    const std::string& tip_frame) = 0;

  /**
   * @brief Forward kinematics: joint state to tip pose.
   * @param joints Joint configuration.
   * @param tip_pose Output tip pose in the base frame.
   * @return true if FK succeeds.
   */
  virtual bool GetPositionFK(
      const automsgs::msgs::sensor_msgs::JointState& joints,
      automsgs::msgs::geometry_msgs::Pose* tip_pose) const = 0;

  /**
   * @brief Inverse kinematics with default InverseKinematicsOptions.
   * @param tip_pose Desired tip pose.
   * @param seed Seed joint state.
   * @param solution Output joint solution.
   * @return true if a solution is found.
   */
  virtual bool GetPositionIK(
      const automsgs::msgs::geometry_msgs::Pose& tip_pose,
      const automsgs::msgs::sensor_msgs::JointState& seed,
      automsgs::msgs::sensor_msgs::JointState* solution) const {
    InverseKinematicsOptions options;
    options.set_timeout(0.05);
    options.set_max_attempts(8);
    return GetPositionIK(tip_pose, seed, options, solution) == ErrorCode::SUCCESS;
  }

  /**
   * @brief Inverse kinematics with explicit options.
   * @param tip_pose Desired tip pose.
   * @param seed Seed joint state.
   * @param options IK timeout / attempt limits.
   * @param solution Output joint solution.
   * @return ErrorCode::SUCCESS on success.
   */
  virtual ErrorCode GetPositionIK(
      const automsgs::msgs::geometry_msgs::Pose& tip_pose,
      const automsgs::msgs::sensor_msgs::JointState& seed,
      const InverseKinematicsOptions& options,
      automsgs::msgs::sensor_msgs::JointState* solution) const = 0;

  /**
   * @brief Multi-seed IK search within options.timeout().
   * @param tip_pose Desired tip pose.
   * @param seed Seed joint state.
   * @param options IK timeout / attempt limits.
   * @param solution Output joint solution.
   * @return ErrorCode::SUCCESS on success.
   */
  virtual ErrorCode SearchPositionIK(
      const automsgs::msgs::geometry_msgs::Pose& tip_pose,
      const automsgs::msgs::sensor_msgs::JointState& seed,
      const InverseKinematicsOptions& options,
      automsgs::msgs::sensor_msgs::JointState* solution) const;

protected:
  /**
   * @brief Default constructor for plugin registration only.
   */
  KinematicsInterface() = default;
};

}  // namespace common
}  // namespace manipulation
}  // namespace autonomy
