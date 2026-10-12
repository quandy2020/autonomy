/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *
 * Adapted from rviz_default_plugins robot::RobotElementBaseClass
 * (Willow Garage / Bosch / Open Source Robotics Foundation).
 *****************************************************************************/

#include "autoviz/plugins/robot/robot_element_base_class.hpp"

namespace autoviz {
namespace display {
namespace robot {

RobotElementBaseClass::RobotElementBaseClass(Robot* robot, std::string name)
    : robot_(robot), name_(std::move(name)) {}

void RobotElementBaseClass::setPose(const QVector3D& position,
                                    const QQuaternion& orientation) {
  position_ = position;
  orientation_ = orientation;
}

}  // namespace robot
}  // namespace display
}  // namespace autoviz
