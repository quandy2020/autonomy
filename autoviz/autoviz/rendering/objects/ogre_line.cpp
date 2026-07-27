/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

#include "autoviz/rendering/objects/ogre_line.hpp"

#ifdef AUTOVIZ_USE_OGRE

#include <OgreManualObject.h>
#include <OgreSceneManager.h>
#include <OgreSceneNode.h>
#include <OgreTechnique.h>

#include "autoviz/rendering/ogre_material_manager.hpp"

namespace autoviz {
namespace rendering {

OgreLine::OgreLine(Ogre::SceneManager* scene_manager,
                   Ogre::SceneNode* parent_node)
    : scene_manager_(scene_manager) {
  if (parent_node == nullptr) {
    parent_node = scene_manager_->getRootSceneNode();
  }
  manual_object_ = scene_manager_->createManualObject();
  scene_node_ = parent_node->createChildSceneNode();
  scene_node_->attachObject(manual_object_);

  static int count = 0;
  const std::string material_name = "AvizLineMaterial" + std::to_string(count++);
  material_ = OgreMaterialManager::createMaterialWithLighting(material_name);
  material_->getTechnique(0)->getPass(0)->setDiffuse(0, 0, 0, 0);
  material_->getTechnique(0)->getPass(0)->setAmbient(1, 1, 1);
}

OgreLine::~OgreLine() {
  if (scene_manager_ == nullptr) {
    return;
  }
  if (scene_node_ != nullptr) {
    if (scene_node_->getParentSceneNode() != nullptr) {
      scene_node_->getParentSceneNode()->removeChild(scene_node_);
    }
    scene_manager_->destroySceneNode(scene_node_);
  }
  if (manual_object_ != nullptr) {
    scene_manager_->destroyManualObject(manual_object_);
  }
  if (material_) {
    material_->unload();
  }
}

void OgreLine::setPoints(Ogre::Vector3 start, Ogre::Vector3 end) {
  manual_object_->clear();
  manual_object_->begin("Autoviz/LineNoLighting",
                        Ogre::RenderOperation::OT_LINE_LIST);
  manual_object_->position(start);
  manual_object_->colour(line_color_);
  manual_object_->position(end);
  manual_object_->colour(line_color_);
  manual_object_->end();
  setVisible(true);
}

void OgreLine::setVisible(bool visible) {
  scene_node_->setVisible(visible, true);
}

void OgreLine::setPosition(const Ogre::Vector3& position) {
  scene_node_->setPosition(position);
}

void OgreLine::setOrientation(const Ogre::Quaternion& orientation) {
  scene_node_->setOrientation(orientation);
}

void OgreLine::setScale(const Ogre::Vector3& scale) {
  scene_node_->setScale(scale);
}

void OgreLine::setColor(const Ogre::ColourValue& color) {
  line_color_ = color;
  material_->getTechnique(0)->setAmbient(color * 0.5f);
  material_->getTechnique(0)->setDiffuse(color);
  OgreMaterialManager::enableAlphaBlending(material_, color.a);
}

void OgreLine::setColor(float r, float g, float b, float a) {
  setColor(Ogre::ColourValue(r, g, b, a));
}

const Ogre::Vector3& OgreLine::position() const {
  return scene_node_->getPosition();
}

const Ogre::Quaternion& OgreLine::orientation() const {
  return scene_node_->getOrientation();
}

void OgreLine::setUserData(const Ogre::Any& data) {
  manual_object_->getUserObjectBindings().setUserAny(data);
}

}  // namespace rendering
}  // namespace autoviz

#endif
