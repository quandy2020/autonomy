/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

#include "autoviz/rendering/objects/ogre_shape.hpp"

#ifdef AUTOVIZ_USE_OGRE

#include <Ogre.h>

#include "autoviz/rendering/ogre_material_manager.hpp"
#include "autoviz/rendering/ogre_procedural_shape.hpp"

namespace autoviz {
namespace rendering {

Ogre::Entity* OgreShape::createEntity(const std::string& name, Type shape_type,
                                      Ogre::SceneManager* scene_manager) {
  ensureRvizPrimitiveMeshes();
  std::string mesh_name;
  switch (shape_type) {
    case kCone:
      mesh_name = "rviz_cone.mesh";
      break;
    case kCube:
      mesh_name = "rviz_cube.mesh";
      break;
    case kCylinder:
      mesh_name = "rviz_cylinder.mesh";
      break;
    case kSphere:
      mesh_name = "rviz_sphere.mesh";
      break;
    case kCapsule:
      mesh_name = "rviz_capsule.mesh";
      break;
  }
  return scene_manager->createEntity(
      name, mesh_name, "rviz_rendering");
}

OgreShape::OgreShape(Type shape_type, Ogre::SceneManager* scene_manager,
                     Ogre::SceneNode* parent_node)
    : scene_manager_(scene_manager), type_(shape_type) {
  static uint32_t count = 0;
  const std::string entity_name = "AvizShape" + std::to_string(count++);

  entity_ = createEntity(entity_name, shape_type, scene_manager_);
  if (parent_node == nullptr) {
    parent_node = scene_manager_->getRootSceneNode();
  }
  scene_node_ = parent_node->createChildSceneNode();
  offset_node_ = scene_node_->createChildSceneNode();
  offset_node_->attachObject(entity_);

  material_name_ = entity_name + "Material";
  material_ = OgreMaterialManager::createMaterialWithLighting(material_name_);
  material_->getTechnique(0)->setAmbient(0.5, 0.5, 0.5);
  entity_->setMaterialName(material_name_, "AvizOgre");
}

OgreShape::~OgreShape() {
  if (scene_manager_ == nullptr) {
    return;
  }
  if (scene_node_ != nullptr) {
    scene_manager_->destroySceneNode(scene_node_);
  }
  if (entity_ != nullptr) {
    scene_manager_->destroyEntity(entity_);
  }
  if (material_) {
    material_->unload();
    Ogre::MaterialManager::getSingleton().remove(material_name_, "AvizOgre");
  }
}

void OgreShape::setOffset(const Ogre::Vector3& offset) {
  offset_node_->setPosition(offset);
}

void OgreShape::setColor(const Ogre::ColourValue& color) {
  material_->getTechnique(0)->setAmbient(color * 0.5f);
  material_->getTechnique(0)->setDiffuse(color);
  OgreMaterialManager::enableAlphaBlending(material_, color.a);
}

void OgreShape::setColor(float r, float g, float b, float a) {
  setColor(Ogre::ColourValue(r, g, b, a));
}

void OgreShape::setPosition(const Ogre::Vector3& position) {
  scene_node_->setPosition(position);
}

void OgreShape::setOrientation(const Ogre::Quaternion& orientation) {
  scene_node_->setOrientation(orientation);
}

void OgreShape::setScale(const Ogre::Vector3& scale) {
  scene_node_->setScale(scale);
}

const Ogre::Vector3& OgreShape::position() const {
  return scene_node_->getPosition();
}

const Ogre::Quaternion& OgreShape::orientation() const {
  return scene_node_->getOrientation();
}

void OgreShape::setUserData(const Ogre::Any& data) {
  if (entity_ != nullptr) {
    entity_->getUserObjectBindings().setUserAny(data);
  }
}

}  // namespace rendering
}  // namespace autoviz

#endif
