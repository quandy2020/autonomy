/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *
 * Adapted from rviz_default_plugins robot::RobotElementBaseClass
 * (Willow Garage / Bosch / Open Source Robotics Foundation).
 *****************************************************************************/

/**
 * @file robot_element_base_class.hpp
 * @brief Shared name / pose state for @ref RobotLink and @ref RobotJoint.
 *
 * RViz attaches property-tree widgets here; Autoviz keeps the pose fields and
 * enable query without Qt property editors.
 *
 * @see RobotLink
 * @see RobotJoint
 */

#pragma once

#include <string>

#include <QQuaternion>
#include <QVector3D>

namespace autoviz {
namespace display {
namespace robot {

class Robot;

/**
 * @class RobotElementBaseClass
 * @brief Named robot part with a cached world pose.
 */
class RobotElementBaseClass {
 public:
  RobotElementBaseClass(Robot* robot, std::string name);
  virtual ~RobotElementBaseClass() = default;

  const std::string& getName() const { return name_; }
  Robot* getRobot() const { return robot_; }

  QVector3D getPosition() const { return position_; }
  QQuaternion getOrientation() const { return orientation_; }

  virtual bool getEnabled() const = 0;

 protected:
  void setPose(const QVector3D& position, const QQuaternion& orientation);

  Robot* robot_ = nullptr;
  std::string name_;
  QVector3D position_;
  QQuaternion orientation_ = QQuaternion(1.f, 0.f, 0.f, 0.f);
};

}  // namespace robot
}  // namespace display
}  // namespace autoviz
