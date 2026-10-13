/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

#include "autoviz/rendering/objects/ogre_arrow.hpp"

#include <OgreSceneManager.h>
#include <OgreSceneNode.h>

#include "autoviz/rendering/objects/ogre_shape.hpp"

namespace autoviz {
namespace rendering {

OgreArrow::OgreArrow(Ogre::SceneManager* scene_manager,
                     Ogre::SceneNode* parent_node, float shaft_length,
                     float shaft_diameter, float head_length,
                     float head_diameter)
    : OgreObject(scene_manager) {
  if (parent_node == nullptr) {
    parent_node = scene_manager_->getRootSceneNode();
  }
  scene_node_ = parent_node->createChildSceneNode();
  shaft_ = new OgreShape(OgreShape::kCylinder, scene_manager_, scene_node_);
  head_ = new OgreShape(OgreShape::kCone, scene_manager_, scene_node_);
  head_->setOffset(Ogre::Vector3(0.f, 0.5f, 0.f));
  set(shaft_length, shaft_diameter, head_length, head_diameter);
  setOrientation(Ogre::Quaternion::IDENTITY);
}

OgreArrow::~OgreArrow() {
  delete shaft_;
  delete head_;
  if (scene_manager_ != nullptr && scene_node_ != nullptr) {
    scene_manager_->destroySceneNode(scene_node_);
    scene_node_ = nullptr;
  }
}

void OgreArrow::set(float shaft_length, float shaft_diameter, float head_length,
                    float head_diameter) {
  shaft_->setScale(Ogre::Vector3(shaft_diameter, shaft_length, shaft_diameter));
  shaft_->setPosition(Ogre::Vector3(0.f, shaft_length / 2.f, 0.f));
  head_->setScale(Ogre::Vector3(head_diameter, head_length, head_diameter));
  head_->setPosition(Ogre::Vector3(0.f, shaft_length, 0.f));
}

void OgreArrow::setColor(const Ogre::ColourValue& color) {
  setShaftColor(color);
  setHeadColor(color);
}

void OgreArrow::setColor(float r, float g, float b, float a) {
  setColor(Ogre::ColourValue(r, g, b, a));
}

void OgreArrow::setShaftColor(const Ogre::ColourValue& color) {
  shaft_->setColor(color);
}

void OgreArrow::setHeadColor(const Ogre::ColourValue& color) {
  head_->setColor(color);
}

void OgreArrow::setShaftColor(float r, float g, float b, float a) {
  setShaftColor(Ogre::ColourValue(r, g, b, a));
}

void OgreArrow::setHeadColor(float r, float g, float b, float a) {
  setHeadColor(Ogre::ColourValue(r, g, b, a));
}

void OgreArrow::setOrientation(const Ogre::Quaternion& orientation) {
  // "forward" (negative z) is the identity orientation; the cylinder and cone
  // meshes point along +Y, so the orientation has to be adjusted.
  scene_node_->setOrientation(
      orientation *
      Ogre::Quaternion(Ogre::Degree(-90.f), Ogre::Vector3::UNIT_X));
}

void OgreArrow::setPosition(const Ogre::Vector3& position) {
  scene_node_->setPosition(position);
}

void OgreArrow::setDirection(const Ogre::Vector3& direction) {
  if (!direction.isZeroLength()) {
    setOrientation(Ogre::Vector3::NEGATIVE_UNIT_Z.getRotationTo(direction));
  }
}

void OgreArrow::setScale(const Ogre::Vector3& scale) {
  // Mangle the scale because of the default orientation of the cylinders.
  scene_node_->setScale(Ogre::Vector3(scale.z, scale.x, scale.y));
}

const Ogre::Vector3& OgreArrow::getPosition() { return scene_node_->getPosition(); }

const Ogre::Quaternion& OgreArrow::getOrientation() {
  return scene_node_->getOrientation();
}

const Ogre::Vector3& OgreArrow::position() const {
  return scene_node_->getPosition();
}

const Ogre::Quaternion& OgreArrow::orientation() const {
  return scene_node_->getOrientation();
}

void OgreArrow::setUserData(const Ogre::Any& data) {
  shaft_->setUserData(data);
  head_->setUserData(data);
}

}  // namespace rendering
}  // namespace autoviz

