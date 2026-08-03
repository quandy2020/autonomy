/******************************************************************************
 * Copyright 2019, Martin Idel · Adapted for Autoviz (BSD-3-Clause).
 *****************************************************************************/

#include "autoviz/rendering/objects/ogre_wrench_visual.hpp"

#ifdef AUTOVIZ_USE_OGRE

#include <algorithm>
#include <cmath>
#include <stdexcept>

#include <OgreSceneManager.h>
#include <OgreSceneNode.h>

#include "autoviz/rendering/objects/ogre_arrow.hpp"
#include "autoviz/rendering/objects/ogre_billboard_line.hpp"

namespace autoviz {
namespace rendering {

OgreWrenchVisual::OgreWrenchVisual(Ogre::SceneManager* scene_manager,
                                    Ogre::SceneNode* parent_node)
    : scene_manager_(scene_manager) {
  if (scene_manager_ == nullptr || parent_node == nullptr) {
    throw std::invalid_argument("Scene manager or parent node is null");
  }
  frame_node_ = parent_node->createChildSceneNode();
  force_node_ = frame_node_->createChildSceneNode();
  torque_node_ = frame_node_->createChildSceneNode();
  arrow_force_ = std::make_unique<OgreArrow>(scene_manager_, force_node_);
  arrow_torque_ = std::make_unique<OgreArrow>(scene_manager_, torque_node_);
  circle_torque_ = std::make_unique<OgreBillboardLine>(scene_manager_, torque_node_);
  circle_arrow_torque_ = std::make_unique<OgreArrow>(scene_manager_, torque_node_);
}

OgreWrenchVisual::~OgreWrenchVisual() {
  arrow_force_.reset();
  arrow_torque_.reset();
  circle_torque_.reset();
  circle_arrow_torque_.reset();
  if (scene_manager_ != nullptr && frame_node_ != nullptr) {
    scene_manager_->destroySceneNode(frame_node_);
    frame_node_ = nullptr;
  }
}

void OgreWrenchVisual::setWrench(const Ogre::Vector3& force, const Ogre::Vector3& torque) {
  force_arrow_direction_ = force;
  torque_arrow_direction_ = torque;
  updateForceArrow();
  updateTorque();
}

void OgreWrenchVisual::updateForceArrow() const {
  const float force_arrow_length = force_arrow_direction_.length() * force_scale_;
  const bool show_force = force_arrow_length > width_;
  if (show_force) {
    arrow_force_->setScale(Ogre::Vector3(force_arrow_length, width_, width_));
    arrow_force_->setDirection(force_arrow_direction_);
  }
  force_node_->setVisible(show_force);
}

void OgreWrenchVisual::updateTorque() const {
  const float torque_arrow_length = torque_arrow_direction_.length() * torque_scale_;
  const bool show_torque = torque_arrow_length > width_;
  if (show_torque) {
    arrow_torque_->setScale(Ogre::Vector3(torque_arrow_length, width_, width_));
    arrow_torque_->setDirection(torque_arrow_direction_);
    const Ogre::Vector3 axis_z(0, 0, 1);
    const Ogre::Quaternion orientation =
        getDirectionOfRotationRelativeToTorque(torque_arrow_direction_, axis_z);
    setTorqueDirectionArrow(orientation);
    createTorqueDirectionCircle(orientation);
  }
  torque_node_->setVisible(show_torque);
}

Ogre::Quaternion OgreWrenchVisual::getDirectionOfRotationRelativeToTorque(
    const Ogre::Vector3& torque, const Ogre::Vector3& axis_z) const {
  Ogre::Quaternion orientation = axis_z.getRotationTo(torque);
  if (std::isnan(orientation.x()) || std::isnan(orientation.y()) || std::isnan(orientation.z())) {
    orientation = Ogre::Quaternion::IDENTITY;
  }
  return orientation;
}

void OgreWrenchVisual::setTorqueDirectionArrow(const Ogre::Quaternion& orientation) const {
  const float torque_arrow_length = torque_arrow_direction_.length() * torque_scale_;
  circle_arrow_torque_->set(0, width_ * 0.1f, width_ * 0.1f * 1.0f, width_ * 0.1f * 2.0f);
  circle_arrow_torque_->setDirection(orientation * Ogre::Vector3(0, 1, 0));
  circle_arrow_torque_->setPosition(
      orientation * Ogre::Vector3(torque_arrow_length / 4, 0, torque_arrow_length / 2));
}

void OgreWrenchVisual::createTorqueDirectionCircle(const Ogre::Quaternion& orientation) const {
  const float torque_arrow_length = torque_arrow_direction_.length() * torque_scale_;
  circle_torque_->clear();
  circle_torque_->setLineWidth(width_ * 0.05f);
  for (int i = 4; i <= 32; ++i) {
    const Ogre::Vector3 point(
        static_cast<float>((torque_arrow_length / 4) * cos(i * 2 * M_PI / 32)),
        static_cast<float>((torque_arrow_length / 4) * sin(i * 2 * M_PI / 32)),
        torque_arrow_length / 2);
    circle_torque_->addPoint(orientation * point);
  }
}

void OgreWrenchVisual::setFramePosition(const Ogre::Vector3& position) {
  frame_node_->setPosition(position);
}

void OgreWrenchVisual::setFrameOrientation(const Ogre::Quaternion& orientation) {
  frame_node_->setOrientation(orientation);
}

void OgreWrenchVisual::setForceColor(float r, float g, float b, float a) {
  arrow_force_->setColor(std::clamp(r, 0.f, 1.f), std::clamp(g, 0.f, 1.f),
                         std::clamp(b, 0.f, 1.f), std::clamp(a, 0.f, 1.f));
}

void OgreWrenchVisual::setTorqueColor(float r, float g, float b, float a) {
  r = std::clamp(r, 0.f, 1.f);
  g = std::clamp(g, 0.f, 1.f);
  b = std::clamp(b, 0.f, 1.f);
  a = std::clamp(a, 0.f, 1.f);
  arrow_torque_->setColor(r, g, b, a);
  circle_torque_->setColor(r, g, b, a);
  circle_arrow_torque_->setColor(r, g, b, a);
}

void OgreWrenchVisual::setForceScale(float scale) {
  force_scale_ = scale;
  updateForceArrow();
}

void OgreWrenchVisual::setTorqueScale(float scale) {
  torque_scale_ = scale;
  updateTorque();
}

void OgreWrenchVisual::setWidth(float width) {
  width_ = width;
  updateForceArrow();
  updateTorque();
}

void OgreWrenchVisual::setVisible(bool visible) { frame_node_->setVisible(visible); }

}  // namespace rendering
}  // namespace autoviz

#endif
