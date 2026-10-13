/******************************************************************************
 * Copyright 2008, Willow Garage, Inc. · Copyright 2017, Open Source Robotics
 * Foundation, Inc. · Adapted for Autoviz (BSD-3-Clause).
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *   * Redistributions of source code must retain the above copyright notice,
 *     this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above copyright notice,
 *     this list of conditions and the following disclaimer in the documentation
 *     and/or other materials provided with the distribution.
 *   * Neither the name of the copyright holder nor the names of its
 *     contributors may be used to endorse or promote products derived from this
 *     software without specific prior written permission.
 * THIS SOFTWARE IS PROVIDED "AS IS" WITHOUT WARRANTY OF ANY KIND; see the
 * RViz BSD-3-Clause license for the full disclaimer.
 *****************************************************************************/


#include "autoviz/rendering/objects/ogre_axes.hpp"

#include <memory>

#include <OgreQuaternion.h>
#include <OgreSceneManager.h>
#include <OgreSceneNode.h>
#include <OgreVector.h>

namespace autoviz {
namespace rendering {

OgreAxes::OgreAxes(Ogre::SceneManager* scene_manager,
                   Ogre::SceneNode* parent_node, float length, float radius)
    : OgreObject(scene_manager) {
  if (parent_node == nullptr) {
    parent_node = scene_manager_->getRootSceneNode();
  }

  scene_node_ = parent_node->createChildSceneNode();

  x_axis_ = std::make_unique<OgreShape>(OgreShape::kCylinder, scene_manager_,
                                        scene_node_);
  y_axis_ = std::make_unique<OgreShape>(OgreShape::kCylinder, scene_manager_,
                                        scene_node_);
  z_axis_ = std::make_unique<OgreShape>(OgreShape::kCylinder, scene_manager_,
                                        scene_node_);

  set(length, radius);
}

OgreAxes::~OgreAxes() {
  // Release shapes (which own child nodes) before destroying the parent node.
  x_axis_.reset();
  y_axis_.reset();
  z_axis_.reset();
  if (scene_manager_ != nullptr && scene_node_ != nullptr) {
    scene_manager_->destroySceneNode(scene_node_);
    scene_node_ = nullptr;
  }
}

void OgreAxes::set(float length, float radius) {
  x_axis_->setScale(Ogre::Vector3(radius, length, radius));
  y_axis_->setScale(Ogre::Vector3(radius, length, radius));
  z_axis_->setScale(Ogre::Vector3(radius, length, radius));

  x_axis_->setPosition(Ogre::Vector3(length / 2.0f, 0.0f, 0.0f));
  x_axis_->setOrientation(
      Ogre::Quaternion(Ogre::Degree(-90), Ogre::Vector3::UNIT_Z));
  y_axis_->setPosition(Ogre::Vector3(0.0f, length / 2.0f, 0.0f));
  z_axis_->setPosition(Ogre::Vector3(0.0f, 0.0f, length / 2.0f));
  z_axis_->setOrientation(
      Ogre::Quaternion(Ogre::Degree(90), Ogre::Vector3::UNIT_X));

  setToDefaultColors();
}

void OgreAxes::setPosition(const Ogre::Vector3& position) {
  scene_node_->setPosition(position);
}

void OgreAxes::setOrientation(const Ogre::Quaternion& orientation) {
  scene_node_->setOrientation(orientation);
}

void OgreAxes::setScale(const Ogre::Vector3& scale) {
  scene_node_->setScale(scale);
}

void OgreAxes::setColor(float r, float g, float b, float a) {
  (void)r;
  (void)g;
  (void)b;
  (void)a;
  // We have several colors, so "setColor" doesn't make sense - noop.
}

const Ogre::Vector3& OgreAxes::getPosition() {
  return scene_node_->getPosition();
}

const Ogre::Quaternion& OgreAxes::getOrientation() {
  return scene_node_->getOrientation();
}

void OgreAxes::setUserData(const Ogre::Any& data) {
  x_axis_->setUserData(data);
  y_axis_->setUserData(data);
  z_axis_->setUserData(data);
}

void OgreAxes::setXColor(const Ogre::ColourValue& col) {
  x_axis_->setColor(col.r, col.g, col.b, col.a);
}

void OgreAxes::setYColor(const Ogre::ColourValue& col) {
  y_axis_->setColor(col.r, col.g, col.b, col.a);
}

void OgreAxes::setZColor(const Ogre::ColourValue& col) {
  z_axis_->setColor(col.r, col.g, col.b, col.a);
}

void OgreAxes::setToDefaultColors() {
  x_axis_->setColor(1.0f, 0.0f, 0.0f, 1.0f);
  y_axis_->setColor(0.0f, 1.0f, 0.0f, 1.0f);
  z_axis_->setColor(0.0f, 0.0f, 1.0f, 1.0f);
}

const Ogre::ColourValue OgreAxes::default_x_color_(1, 0, 0, 1);
const Ogre::ColourValue OgreAxes::default_y_color_(0, 1, 0, 1);
const Ogre::ColourValue OgreAxes::default_z_color_(0, 0, 1, 1);

const Ogre::ColourValue& OgreAxes::getDefaultXColor() { return default_x_color_; }

const Ogre::ColourValue& OgreAxes::getDefaultYColor() { return default_y_color_; }

const Ogre::ColourValue& OgreAxes::getDefaultZColor() { return default_z_color_; }

}  // namespace rendering
}  // namespace autoviz
