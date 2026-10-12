/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *
 * Adapted from rviz_default_plugins robot::RobotLink
 * (Willow Garage / Bosch / Open Source Robotics Foundation).
 *****************************************************************************/

/**
 * @file robot_link.hpp
 * @brief One URDF link: enable flags + visual/collision world pose.
 *
 * Geometry meshes stay owned by @ref Robot (Autoviz SceneOverlay / Ogre path).
 * This class mirrors RViz's per-link transform / visibility state.
 *
 * @see Robot
 * @see LinkUpdater
 */

#pragma once

#include <string>

#include <QMatrix4x4>
#include <QQuaternion>
#include <QVector3D>

#include "autoviz/plugins/robot/robot_element_base_class.hpp"

namespace autoviz {
namespace display {
namespace robot {

class Robot;

/**
 * @class RobotLink
 * @brief Link visibility and transforms updated by @ref Robot::update.
 */
class RobotLink : public RobotElementBaseClass {
 public:
  RobotLink(Robot* robot, std::string name);

  bool getEnabled() const override { return enabled_; }
  void setEnabled(bool enabled) { enabled_ = enabled; }

  void setRobotAlpha(float alpha) { robot_alpha_ = alpha; }
  float getRobotAlpha() const { return robot_alpha_; }

  /**
   * @brief Stores visual and collision poses from a @ref LinkUpdater.
   */
  void setTransforms(const QVector3D& visual_position,
                     const QQuaternion& visual_orientation,
                     const QVector3D& collision_position,
                     const QQuaternion& collision_orientation);

  /** Clears the last TF/FK pose (forces draw fallback). */
  void clearTransform();

  bool hasTransform() const { return have_transform_; }

  QVector3D getVisualPosition() const { return visual_position_; }
  QQuaternion getVisualOrientation() const { return visual_orientation_; }
  QVector3D getCollisionPosition() const { return collision_position_; }
  QQuaternion getCollisionOrientation() const {
    return collision_orientation_;
  }

  /** World matrix for visual geometry (translation × rotation). */
  QMatrix4x4 visualWorldMatrix() const;
  QMatrix4x4 collisionWorldMatrix() const;

 private:
  bool enabled_ = true;
  bool have_transform_ = false;
  float robot_alpha_ = 1.f;
  QVector3D visual_position_;
  QQuaternion visual_orientation_ = QQuaternion(1.f, 0.f, 0.f, 0.f);
  QVector3D collision_position_;
  QQuaternion collision_orientation_ = QQuaternion(1.f, 0.f, 0.f, 0.f);
};

}  // namespace robot
}  // namespace display
}  // namespace autoviz
