/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *
 * Adapted from rviz_default_plugins robot::RobotLink
 * (Willow Garage / Bosch / Open Source Robotics Foundation).
 *****************************************************************************/

#include "autoviz/plugins/robot/robot_link.hpp"

namespace autoviz {
namespace display {
namespace robot {
namespace {

QMatrix4x4 MatrixFromPose(const QVector3D& position,
                          const QQuaternion& orientation) {
  QMatrix4x4 matrix;
  matrix.setToIdentity();
  matrix.translate(position);
  matrix.rotate(orientation);
  return matrix;
}

}  // namespace

RobotLink::RobotLink(Robot* robot, std::string name)
    : RobotElementBaseClass(robot, std::move(name)) {}

void RobotLink::setTransforms(const QVector3D& visual_position,
                              const QQuaternion& visual_orientation,
                              const QVector3D& collision_position,
                              const QQuaternion& collision_orientation) {
  visual_position_ = visual_position;
  visual_orientation_ = visual_orientation;
  collision_position_ = collision_position;
  collision_orientation_ = collision_orientation;
  setPose(visual_position, visual_orientation);
  have_transform_ = true;
}

void RobotLink::clearTransform() { have_transform_ = false; }

QMatrix4x4 RobotLink::visualWorldMatrix() const {
  return MatrixFromPose(visual_position_, visual_orientation_);
}

QMatrix4x4 RobotLink::collisionWorldMatrix() const {
  return MatrixFromPose(collision_position_, collision_orientation_);
}

}  // namespace robot
}  // namespace display
}  // namespace autoviz
