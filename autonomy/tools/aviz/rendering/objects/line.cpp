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

#include "autonomy/tools/aviz/rendering/objects/line.hpp"

#include <OGRE/OgreManualObject.h>
#include <OGRE/OgreMaterialManager.h>
#include <OGRE/OgreSceneManager.h>
#include <OGRE/OgreSceneNode.h>
#include <OGRE/OgreTechnique.h>

#include <string>

#define ROS_PACKAGE_NAME "aviz_rendering"

namespace aviz {
namespace rendering {

Line::Line(Ogre::SceneManager* scene_manager, Ogre::SceneNode* parent_node) : Object(scene_manager) {
  if (!parent_node) {
    parent_node = scene_manager_->getRootSceneNode();
  }
  manual_object_ = scene_manager_->createManualObject();
  scene_node_ = parent_node->createChildSceneNode();

  static int count = 0;
  std::string line_material_name = "LineMaterial" + std::to_string(count++);

  // Create material with lighting
  manual_object_material_ = Ogre::MaterialManager::getSingleton().create(line_material_name, ROS_PACKAGE_NAME);
  Ogre::Technique* technique = manual_object_material_->getTechnique(0);
  if (technique) {
    Ogre::Pass* pass = technique->getPass(0);
    if (pass) {
      pass->setDiffuse(0, 0, 0, 0);
      pass->setAmbient(1, 1, 1);
      pass->setLightingEnabled(true);
    }
  }

  scene_node_->attachObject(manual_object_);
}

Line::~Line() {
  if (scene_node_->getParentSceneNode()) {
    scene_node_->getParentSceneNode()->removeChild(scene_node_);
  }
  scene_manager_->destroySceneNode(scene_node_);
  scene_manager_->destroyManualObject(manual_object_);
  manual_object_material_->unload();
  Ogre::MaterialManager::getSingleton().remove(manual_object_material_->getHandle());
}

void Line::setPoints(Ogre::Vector3 start, Ogre::Vector3 end) {
  manual_object_->clear();
  manual_object_->begin(manual_object_material_->getName(), Ogre::RenderOperation::OT_LINE_LIST, ROS_PACKAGE_NAME);
  manual_object_->position(start);
  manual_object_->position(end);
  manual_object_->end();
  setVisible(true);
}

void Line::setVisible(bool visible) { scene_node_->setVisible(visible, true); }

void Line::setPosition(const Ogre::Vector3& position) { scene_node_->setPosition(position); }

void Line::setOrientation(const Ogre::Quaternion& orientation) { scene_node_->setOrientation(orientation); }

void Line::setScale(const Ogre::Vector3& scale) { scene_node_->setScale(scale); }

void Line::setColor(const Ogre::ColourValue& c) {
  Ogre::Technique* technique = manual_object_material_->getTechnique(0);
  if (technique) {
    Ogre::Pass* pass = technique->getPass(0);
    if (pass) {
      pass->setDiffuse(c);
      pass->setAmbient(c * 0.5);
    }
  }
}

void Line::setColor(float r, float g, float b, float a) { setColor(Ogre::ColourValue(r, g, b, a)); }

const Ogre::Vector3& Line::getPosition() { return scene_node_->getPosition(); }

const Ogre::Quaternion& Line::getOrientation() { return scene_node_->getOrientation(); }

void Line::setUserData(const Ogre::Any& data) { manual_object_->getUserObjectBindings().setUserAny(data); }

}  // namespace rendering
}  // namespace aviz
