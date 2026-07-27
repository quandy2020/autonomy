/******************************************************************************
 * Copyright 2008, Willow Garage, Inc.
 * Adapted for Autoviz (BSD-3-Clause).
 *****************************************************************************/

#include "autoviz/rendering/objects/ogre_mesh_shape.hpp"

#ifdef AUTOVIZ_USE_OGRE

#include <OgreEntity.h>
#include <OgreMaterialManager.h>
#include <OgreMesh.h>
#include <OgreMeshManager.h>

#include "autoviz/rendering/ogre_logging.hpp"
#include "autoviz/rendering/ogre_material_manager.hpp"

namespace autoviz {
namespace rendering {

OgreMeshShape::OgreMeshShape(Ogre::SceneManager* scene_manager,
                             Ogre::SceneNode* parent_node)
    : scene_manager_(scene_manager) {
  static uint32_t count = 0;
  manual_object_ =
      scene_manager_->createManualObject("OgreMeshShape_ManualObject" +
                                         std::to_string(count++));
  if (parent_node == nullptr) {
    parent_node = scene_manager_->getRootSceneNode();
  }
  scene_node_ = parent_node->createChildSceneNode();
  offset_node_ = scene_node_->createChildSceneNode();
  material_name_ = "OgreMeshShapeMaterial" + std::to_string(count);
  material_ = OgreMaterialManager::createMaterialWithLighting(material_name_);
  material_->setCullingMode(Ogre::CULL_NONE);
}

OgreMeshShape::~OgreMeshShape() {
  clear();
  if (scene_manager_ != nullptr && manual_object_ != nullptr) {
    scene_manager_->destroyManualObject(manual_object_);
  }
  if (scene_manager_ != nullptr && scene_node_ != nullptr) {
    scene_manager_->destroySceneNode(scene_node_);
  }
  if (material_) {
    material_->unload();
    Ogre::MaterialManager::getSingleton().remove(material_name_, "AvizOgre");
  }
}

void OgreMeshShape::estimateVertexCount(size_t vcount) {
  if (entity_ == nullptr && !started_) {
    manual_object_->estimateVertexCount(vcount);
  }
}

void OgreMeshShape::beginTriangles() {
  if (!started_ && entity_ != nullptr) {
    AUTOVIZ_OGRE_LOG_WARNING("Cannot modify mesh once construction is complete");
    return;
  }
  if (!started_) {
    started_ = true;
    manual_object_->begin(material_name_, Ogre::RenderOperation::OT_TRIANGLE_LIST,
                          "AvizOgre");
  }
}

void OgreMeshShape::addVertex(const Ogre::Vector3& position) {
  beginTriangles();
  manual_object_->position(position);
}

void OgreMeshShape::addVertex(const Ogre::Vector3& position,
                              const Ogre::Vector3& normal) {
  beginTriangles();
  manual_object_->position(position);
  manual_object_->normal(normal);
}

void OgreMeshShape::addVertex(const Ogre::Vector3& position,
                              const Ogre::Vector3& normal,
                              const Ogre::ColourValue& color) {
  beginTriangles();
  manual_object_->position(position);
  manual_object_->normal(normal);
  manual_object_->colour(color);
}

void OgreMeshShape::addNormal(const Ogre::Vector3& normal) {
  manual_object_->normal(normal);
}

void OgreMeshShape::addColor(const Ogre::ColourValue& color) {
  manual_object_->colour(color);
}

void OgreMeshShape::addTriangle(unsigned int v1, unsigned int v2,
                                unsigned int v3) {
  manual_object_->triangle(v1, v2, v3);
}

void OgreMeshShape::endTriangles() {
  if (started_) {
    started_ = false;
    manual_object_->end();
    static uint32_t count = 0;
    const std::string name = "ConvertedOgreMeshShape@" + std::to_string(count++);
    manual_object_->convertToMesh(name);
    entity_ = scene_manager_->createEntity(name);
    if (entity_ != nullptr) {
      entity_->setMaterial(material_);
      offset_node_->attachObject(entity_);
    } else {
      AUTOVIZ_OGRE_LOG_ERROR("Unable to construct triangle mesh");
    }
  } else {
    AUTOVIZ_OGRE_LOG_ERROR("No triangles added");
  }
}

void OgreMeshShape::clear() {
  if (entity_ != nullptr) {
    entity_->detachFromParent();
    Ogre::MeshManager::getSingleton().remove(entity_->getMesh()->getName());
    scene_manager_->destroyEntity(entity_);
    entity_ = nullptr;
  }
  manual_object_->clear();
  started_ = false;
}

}  // namespace rendering
}  // namespace autoviz

#endif
