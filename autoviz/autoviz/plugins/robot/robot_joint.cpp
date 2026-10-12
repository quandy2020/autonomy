/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *
 * Adapted from rviz_default_plugins robot::RobotJoint
 * (Willow Garage / Bosch / Open Source Robotics Foundation).
 *****************************************************************************/

#include "autoviz/plugins/robot/robot_joint.hpp"

namespace autoviz {
namespace display {
namespace robot {

RobotJoint::RobotJoint(Robot* robot, const UrdfJoint& joint)
    : RobotElementBaseClass(robot, joint.name),
      parent_link_name_(joint.parent),
      child_link_name_(joint.child),
      type_(joint.type),
      axis_(joint.axis) {}

}  // namespace robot
}  // namespace display
}  // namespace autoviz
