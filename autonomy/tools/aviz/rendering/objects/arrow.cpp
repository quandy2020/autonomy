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

#include "autonomy/tools/aviz/rendering/objects/arrow.hpp"

#include <OGRE/OgreQuaternion.h>
#include <OGRE/OgreSceneManager.h>
#include <OGRE/OgreSceneNode.h>
#include <OGRE/OgreVector3.h>

#include "autonomy/tools/aviz/rendering/objects/shape.hpp"

namespace aviz {
namespace rendering {

Arrow::Arrow(Ogre::SceneManager* scene_manager, Ogre::SceneNode* parent_node, float shaft_length, float shaft_diameter,
             float head_length, float head_diameter)
    : Object(scene_manager) {
    if (!parent_node) {
        parent_node = scene_manager_->getRootSceneNode();
    }

    scene_node_ = parent_node->createChildSceneNode();

    shaft_ = new Shape(Shape::Cylinder, scene_manager_, scene_node_);
    head_ = new Shape(Shape::Cone, scene_manager_, scene_node_);
    head_->setOffset(Ogre::Vector3(0.0f, 0.5f, 0.0f));

    set(shaft_length, shaft_diameter, head_length, head_diameter);

    setOrientation(Ogre::Quaternion::IDENTITY);
}

Arrow::~Arrow() {
    delete shaft_;
    delete head_;

    scene_manager_->destroySceneNode(scene_node_);
}

void Arrow::set(float shaft_length, float shaft_diameter, float head_length, float head_diameter) {
    shaft_->setScale(Ogre::Vector3(shaft_diameter, shaft_length, shaft_diameter));
    shaft_->setPosition(Ogre::Vector3(0.0f, shaft_length / 2.0f, 0.0f));

    head_->setScale(Ogre::Vector3(head_diameter, head_length, head_diameter));
    head_->setPosition(Ogre::Vector3(0.0f, shaft_length, 0.0f));
}

void Arrow::setColor(const Ogre::ColourValue& c) {
    setShaftColor(c);
    setHeadColor(c);
}

void Arrow::setColor(float r, float g, float b, float a) {
    setColor(Ogre::ColourValue(r, g, b, a));
}

void Arrow::setShaftColor(const Ogre::ColourValue& c) {
    shaft_->setColor(c);
}

void Arrow::setHeadColor(const Ogre::ColourValue& c) {
    head_->setColor(c);
}

void Arrow::setShaftColor(float r, float g, float b, float a) {
    setShaftColor(Ogre::ColourValue(r, g, b, a));
}

void Arrow::setHeadColor(float r, float g, float b, float a) {
    setHeadColor(Ogre::ColourValue(r, g, b, a));
}

void Arrow::setPosition(const Ogre::Vector3& position) {
    scene_node_->setPosition(position);
}

void Arrow::setOrientation(const Ogre::Quaternion& orientation) {
    // "forward" (negative z) should always be our identity orientation
    scene_node_->setOrientation(orientation * Ogre::Quaternion(Ogre::Degree(-90), Ogre::Vector3::UNIT_X));
}

void Arrow::setDirection(const Ogre::Vector3& direction) {
    if (!direction.isZeroLength()) {
        setOrientation(Ogre::Vector3::NEGATIVE_UNIT_Z.getRotationTo(direction));
    }
}

void Arrow::setScale(const Ogre::Vector3& scale) {
    // Have to mangle the scale because of the default orientation of the cylinders
    scene_node_->setScale(Ogre::Vector3(scale.z, scale.x, scale.y));
}

const Ogre::Vector3& Arrow::getPosition() {
    return scene_node_->getPosition();
}

const Ogre::Quaternion& Arrow::getOrientation() {
    return scene_node_->getOrientation();
}

void Arrow::setUserData(const Ogre::Any& data) {
    head_->setUserData(data);
    shaft_->setUserData(data);
}

}  // namespace rendering
}  // namespace aviz
