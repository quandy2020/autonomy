/******************************************************************************
 * Copyright 2023, Open Source Robotics Foundation, Inc.
 * Adapted for Autoviz (BSD-3-Clause).
 *****************************************************************************/

#include "autoviz/rendering/objects/ogre_screw_visual.hpp"

#ifdef AUTOVIZ_USE_OGRE

#include <cmath>
#include <stdexcept>

#include <OgreSceneManager.h>
#include <OgreSceneNode.h>

#include "autoviz/rendering/objects/ogre_arrow.hpp"
#include "autoviz/rendering/objects/ogre_billboard_line.hpp"

namespace autoviz {
namespace rendering {

OgreScrewVisual::OgreScrewVisual(Ogre::SceneManager* scene_manager,
                                 Ogre::SceneNode* parent_node)
    : scene_manager_(scene_manager) {
  if (scene_manager_ == nullptr || parent_node == nullptr) {
    throw std::invalid_argument("Invalid input: scene_manager or parent_node is nullptr.");
  }
  frame_node_ = parent_node->createChildSceneNode();
  linear_node_ = frame_node_->createChildSceneNode();
  angular_node_ = frame_node_->createChildSceneNode();
  arrow_linear_ = std::make_unique<OgreArrow>(scene_manager_, linear_node_);
  arrow_angular_ = std::make_unique<OgreArrow>(scene_manager_, angular_node_);
  circle_angular_ = std::make_unique<OgreBillboardLine>(scene_manager_, angular_node_);
  circle_arrow_angular_ = std::make_unique<OgreArrow>(scene_manager_, angular_node_);
}

OgreScrewVisual::~OgreScrewVisual() {
  arrow_linear_.reset();
  arrow_angular_.reset();
  circle_angular_.reset();
  circle_arrow_angular_.reset();
  if (scene_manager_ != nullptr && frame_node_ != nullptr) {
    scene_manager_->destroySceneNode(frame_node_);
    frame_node_ = nullptr;
  }
}

void OgreScrewVisual::setScrew(const Ogre::Vector3& linear, const Ogre::Vector3& angular) {
  const float linear_length = linear.length() * linear_scale_;
  const float angular_length = angular.length() * angular_scale_;
  const bool show_linear = (linear_length > width_) || !hide_small_values_;
  const bool show_angular = (angular_length > width_) || !hide_small_values_;

  if (show_linear) {
    arrow_linear_->setScale(Ogre::Vector3(linear_length, width_, width_));
    arrow_linear_->setDirection(linear);
  }
  linear_node_->setVisible(show_linear);

  if (show_angular) {
    arrow_angular_->setScale(Ogre::Vector3(angular_length, width_, width_));
    arrow_angular_->setDirection(angular);
    const Ogre::Vector3 axis_z(0, 0, 1);
    Ogre::Quaternion orientation = axis_z.getRotationTo(angular);
    if (std::isnan(orientation.x) || std::isnan(orientation.y) || std::isnan(orientation.z)) {
      orientation = Ogre::Quaternion::IDENTITY;
    }
    circle_arrow_angular_->set(0, width_ * 0.1f, width_ * 0.1f * 1.0f, width_ * 0.1f * 2.0f);
    circle_arrow_angular_->setDirection(orientation * Ogre::Vector3(0, 1, 0));
    circle_arrow_angular_->setPosition(
        orientation * Ogre::Vector3(angular_length / 4.0f, 0, angular_length / 2.0f));
    circle_angular_->clear();
    circle_angular_->setLineWidth(width_ * 0.05f);
    for (int i = 4; i <= 32; ++i) {
      const Ogre::Vector3 point(
          static_cast<float>((angular_length / 4.0f) * cos(i * 2.0f * M_PI / 32.0f)),
          static_cast<float>((angular_length / 4.0f) * sin(i * 2.0f * M_PI / 32.0f)),
          static_cast<float>(angular_length / 2.0f));
      circle_angular_->addPoint(orientation * point);
    }
  }
  angular_node_->setVisible(show_angular);
}

void OgreScrewVisual::setFramePosition(const Ogre::Vector3& position) {
  frame_node_->setPosition(position);
}

void OgreScrewVisual::setFrameOrientation(const Ogre::Quaternion& orientation) {
  frame_node_->setOrientation(orientation);
}

void OgreScrewVisual::setLinearColor(float r, float g, float b, float a) {
  arrow_linear_->setColor(r, g, b, a);
}

void OgreScrewVisual::setAngularColor(float r, float g, float b, float a) {
  arrow_angular_->setColor(r, g, b, a);
  circle_angular_->setColor(r, g, b, a);
  circle_arrow_angular_->setColor(r, g, b, a);
}

void OgreScrewVisual::setLinearScale(float scale) { linear_scale_ = scale; }
void OgreScrewVisual::setAngularScale(float scale) { angular_scale_ = scale; }
void OgreScrewVisual::setWidth(float width) { width_ = width; }
void OgreScrewVisual::setHideSmallValues(bool hide) { hide_small_values_ = hide; }
void OgreScrewVisual::setVisible(bool visible) { frame_node_->setVisible(visible); }

}  // namespace rendering
}  // namespace autoviz

#endif
