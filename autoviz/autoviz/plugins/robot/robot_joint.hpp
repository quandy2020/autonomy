/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *
 * Adapted from rviz_default_plugins robot::RobotJoint
 * (Willow Garage / Bosch / Open Source Robotics Foundation).
 *****************************************************************************/

/**
 * @file robot_joint.hpp
 * @brief One URDF joint: connectivity + type metadata for the robot tree.
 *
 * @see Robot
 * @see RobotLink
 */

#pragma once

#include <string>

#include <QVector3D>

#include "autoviz/plugins/displays/common/urdf_model.hpp"
#include "autoviz/plugins/robot/robot_element_base_class.hpp"

namespace autoviz {
namespace display {
namespace robot {

class Robot;

/**
 * @class RobotJoint
 * @brief Joint description mirrored from @ref UrdfJoint.
 */
class RobotJoint : public RobotElementBaseClass {
 public:
  RobotJoint(Robot* robot, const UrdfJoint& joint);

  bool getEnabled() const override { return enabled_; }
  void setEnabled(bool enabled) { enabled_ = enabled; }

  const std::string& getParentLinkName() const { return parent_link_name_; }
  const std::string& getChildLinkName() const { return child_link_name_; }
  UrdfJointType getType() const { return type_; }
  QVector3D getAxis() const { return axis_; }

  void setPosition(double position) { position_value_ = position; }
  double getPosition() const { return position_value_; }

 private:
  bool enabled_ = true;
  std::string parent_link_name_;
  std::string child_link_name_;
  UrdfJointType type_ = UrdfJointType::kFixed;
  QVector3D axis_{0.f, 0.f, 1.f};
  double position_value_ = 0.0;
};

}  // namespace robot
}  // namespace display
}  // namespace autoviz
