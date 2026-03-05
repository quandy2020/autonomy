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

#include "autonomy/tools/aviz/rendering/objects/shape.hpp"

#include <OGRE/OgreEntity.h>
#include <OGRE/OgreMaterialManager.h>
#include <OGRE/OgreQuaternion.h>
#include <OGRE/OgreResourceGroupManager.h>
#include <OGRE/OgreSceneManager.h>
#include <OGRE/OgreSceneNode.h>
#include <OGRE/OgreTechnique.h>
#include <OGRE/OgreVector3.h>

#include <cstdint>
#include <string>

#define ROS_PACKAGE_NAME "aviz_rendering"

namespace aviz {
namespace rendering {

Ogre::Entity* Shape::createEntity(const std::string& name, Type type, Ogre::SceneManager* scene_manager) {
  if (type == Mesh) {
    return nullptr;  // the entity is initialized after the vertex data was specified
  }
  std::string mesh_name;
  switch (type) {
    case Cone:
      mesh_name = "aviz_cone.mesh";
      break;
    case Cube:
      mesh_name = "aviz_cube.mesh";
      break;
    case Cylinder:
      mesh_name = "aviz_cylinder.mesh";
      break;
    case Sphere:
      mesh_name = "aviz_sphere.mesh";
      break;
    default:
      throw std::runtime_error("unexpected mesh entity type");
  }

  return scene_manager->createEntity(name, mesh_name, Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME);
}

Shape::Shape(Type type, Ogre::SceneManager* scene_manager, Ogre::SceneNode* parent_node)
    : Object(scene_manager), type_(type) {
  static uint32_t count = 0;
  std::string entity_name = "Shape" + std::to_string(count++);

  entity_ = createEntity(entity_name, type, scene_manager);

  if (!parent_node) {
    parent_node = scene_manager_->getRootSceneNode();
  }

  scene_node_ = parent_node->createChildSceneNode();
  offset_node_ = scene_node_->createChildSceneNode();
  if (entity_) {
    offset_node_->attachObject(entity_);
  }

  material_name_ = entity_name + "Material";
  material_ = Ogre::MaterialManager::getSingleton().create(material_name_, ROS_PACKAGE_NAME);
  Ogre::Technique* technique = material_->getTechnique(0);
  if (technique) {
    Ogre::Pass* pass = technique->getPass(0);
    if (pass) {
      pass->setAmbient(0.5, 0.5, 0.5);
      pass->setDiffuse(0.5, 0.5, 0.5, 1.0);
      pass->setSpecular(0.1, 0.1, 0.1, 1.0);
    }
  }

  if (entity_) {
    entity_->setMaterialName(material_name_);
  }

#if (OGRE_VERSION_MAJOR <= 1 && OGRE_VERSION_MINOR <= 4)
  if (entity_) {
    entity_->setNormaliseNormals(true);
  }
#endif
}

Shape::~Shape() {
  scene_manager_->destroySceneNode(scene_node_);
  scene_manager_->destroySceneNode(offset_node_);

  if (entity_) {
    scene_manager_->destroyEntity(entity_);
  }

  material_->unload();
  Ogre::MaterialManager::getSingleton().remove(material_->getHandle());
}

void Shape::setColor(const Ogre::ColourValue& c) {
  Ogre::Technique* technique = material_->getTechnique(0);
  if (technique) {
    Ogre::Pass* pass = technique->getPass(0);
    if (pass) {
      pass->setAmbient(c * 0.5);
      pass->setDiffuse(c);

      if (c.a < 0.9998) {
        pass->setSceneBlending(Ogre::SBT_TRANSPARENT_ALPHA);
        pass->setDepthWriteEnabled(false);
      } else {
        pass->setSceneBlending(Ogre::SBT_REPLACE);
        pass->setDepthWriteEnabled(true);
      }
    }
  }
}

void Shape::setColor(float r, float g, float b, float a) { setColor(Ogre::ColourValue(r, g, b, a)); }

void Shape::setOffset(const Ogre::Vector3& offset) { offset_node_->setPosition(offset); }

void Shape::setPosition(const Ogre::Vector3& position) { scene_node_->setPosition(position); }

void Shape::setOrientation(const Ogre::Quaternion& orientation) { scene_node_->setOrientation(orientation); }

void Shape::setScale(const Ogre::Vector3& scale) { scene_node_->setScale(scale); }

const Ogre::Vector3& Shape::getPosition() { return scene_node_->getPosition(); }

const Ogre::Quaternion& Shape::getOrientation() { return scene_node_->getOrientation(); }

void Shape::setUserData(const Ogre::Any& data) {
  if (entity_) {
    entity_->getUserObjectBindings().setUserAny(data);
  }
}

}  // namespace rendering
}  // namespace aviz
