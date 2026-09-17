/*
 * Copyright 2026 The Openbot Authors
 *
 * Abstract robot kinematic model loaded from URDF/SRDF.
 */

#pragma once

#include <memory>
#include <string>
#include <vector>

#include "autonomy/common/macros.hpp"

#include <automsgs/msgs/moveit_msgs/robot_model.pb.h>

namespace autonomy {
namespace manipulation {
namespace model {

using JointLimits = ::automsgs::msgs::moveit_msgs::JointLimits;
using JointModel = ::automsgs::msgs::moveit_msgs::JointModel;
using LinkModel = ::automsgs::msgs::moveit_msgs::LinkModel;
using JointModelGroup = ::automsgs::msgs::moveit_msgs::JointModelGroup;
using SrdfEndEffector = ::automsgs::msgs::moveit_msgs::SrdfEndEffector;

/**
 * @class RobotModel
 * @brief Abstract robot kinematic model loaded from URDF/SRDF.
 */
class RobotModel {
 public:
  AUTONOMY_SMART_PTR_DEFINITIONS(RobotModel)

  virtual ~RobotModel() = default;

  /**
   * @brief Load model definition from URDF and SRDF paths.
   * @param[in] urdf Path to URDF file.
   * @param[in] srdf Path to SRDF file.
   * @return true on successful load.
   */
  virtual bool Load(const std::string& urdf, const std::string& srdf) = 0;

  /**
   * @brief Joint names belonging to a planning group.
   * @param[in] group Planning group name.
   * @return Ordered joint names for @p group (empty if unknown).
   */
  virtual std::vector<std::string> GetJointNames(
      const std::string& group) const = 0;

  /** @brief All joints in the model. */
  virtual const std::vector<JointModel>& Joints() const = 0;

  /**
   * @brief Look up a planning group by name.
   * @param[in] name Group name.
   * @return Pointer to the group, or nullptr if missing.
   */
  virtual const JointModelGroup* GetJointModelGroup(
      const std::string& name) const = 0;

  /**
   * @brief Look up limits for a joint.
   * @param[in] joint_name Joint name.
   * @return Pointer to limits, or nullptr if missing.
   */
  virtual const JointLimits* GetJointLimits(
      const std::string& joint_name) const = 0;

 protected:
  RobotModel() = default;
};

}  // namespace model
}  // namespace manipulation
}  // namespace autonomy
