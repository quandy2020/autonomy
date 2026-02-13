/******************************************************************************
 * Copyright 2025 The Openbot Authors (duyongquan)
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *****************************************************************************/

#include "autonomy/tools/aviz/rendering/objects/axes.hpp"

#include <memory>

#include <OGRE/OgreQuaternion.h>
#include <OGRE/OgreSceneManager.h>
#include <OGRE/OgreSceneNode.h>
#include <OGRE/OgreVector3.h>

namespace aviz {
namespace rendering {

const Ogre::ColourValue Axes::default_x_color_(1, 0, 0, 1);
const Ogre::ColourValue Axes::default_y_color_(0, 1, 0, 1);
const Ogre::ColourValue Axes::default_z_color_(0, 0, 1, 1);

Axes::Axes(Ogre::SceneManager* scene_manager, Ogre::SceneNode* parent_node, float length, float radius)
    : Object(scene_manager) {
    if (!parent_node) {
        parent_node = scene_manager_->getRootSceneNode();
    }

    scene_node_ = parent_node->createChildSceneNode();

    x_axis_ = std::make_unique<Shape>(Shape::Cylinder, scene_manager_, scene_node_);
    y_axis_ = std::make_unique<Shape>(Shape::Cylinder, scene_manager_, scene_node_);
    z_axis_ = std::make_unique<Shape>(Shape::Cylinder, scene_manager_, scene_node_);

    set(length, radius);
}

Axes::~Axes() {
    scene_manager_->destroySceneNode(scene_node_);
}

void Axes::set(float length, float radius) {
    x_axis_->setScale(Ogre::Vector3(radius, length, radius));
    y_axis_->setScale(Ogre::Vector3(radius, length, radius));
    z_axis_->setScale(Ogre::Vector3(radius, length, radius));

    x_axis_->setPosition(Ogre::Vector3(length / 2.0f, 0.0f, 0.0f));
    x_axis_->setOrientation(Ogre::Quaternion(Ogre::Degree(-90), Ogre::Vector3::UNIT_Z));
    y_axis_->setPosition(Ogre::Vector3(0.0f, length / 2.0f, 0.0f));
    z_axis_->setPosition(Ogre::Vector3(0.0, 0.0f, length / 2.0f));
    z_axis_->setOrientation(Ogre::Quaternion(Ogre::Degree(90), Ogre::Vector3::UNIT_X));

    setToDefaultColors();
}

void Axes::setPosition(const Ogre::Vector3& position) {
    scene_node_->setPosition(position);
}

void Axes::setOrientation(const Ogre::Quaternion& orientation) {
    scene_node_->setOrientation(orientation);
}

void Axes::setScale(const Ogre::Vector3& scale) {
    scene_node_->setScale(scale);
}

void Axes::setColor(float r, float g, float b, float a) {
    (void)r;
    (void)g;
    (void)b;
    (void)a;
    // we have several colors, so "setColor" doesn't make sense - noop
}

const Ogre::Vector3& Axes::getPosition() {
    return scene_node_->getPosition();
}

const Ogre::Quaternion& Axes::getOrientation() {
    return scene_node_->getOrientation();
}

void Axes::setUserData(const Ogre::Any& data) {
    x_axis_->setUserData(data);
    y_axis_->setUserData(data);
    z_axis_->setUserData(data);
}

void Axes::setXColor(const Ogre::ColourValue& col) {
    x_axis_->setColor(col.r, col.g, col.b, col.a);
}

void Axes::setYColor(const Ogre::ColourValue& col) {
    y_axis_->setColor(col.r, col.g, col.b, col.a);
}

void Axes::setZColor(const Ogre::ColourValue& col) {
    z_axis_->setColor(col.r, col.g, col.b, col.a);
}

void Axes::setToDefaultColors() {
    x_axis_->setColor(1.0f, 0.0f, 0.0f, 1.0f);
    y_axis_->setColor(0.0f, 1.0f, 0.0f, 1.0f);
    z_axis_->setColor(0.0f, 0.0f, 1.0f, 1.0f);
}

const Ogre::ColourValue& Axes::getDefaultXColor() {
    return default_x_color_;
}

const Ogre::ColourValue& Axes::getDefaultYColor() {
    return default_y_color_;
}

const Ogre::ColourValue& Axes::getDefaultZColor() {
    return default_z_color_;
}

}  // namespace rendering
}  // namespace aviz
