/******************************************************************************
 * Copyright 2008, Willow Garage, Inc.
 * Adapted for Autoviz (BSD-3-Clause).
 *****************************************************************************/

#pragma once

#ifdef AUTOVIZ_USE_OGRE

#include <cstddef>

#include <OgreColourValue.h>
#include <OgreManualObject.h>
#include <OgreMaterial.h>
#include <OgreSceneManager.h>
#include <OgreSceneNode.h>
#include <OgreSharedPtr.h>
#include <OgreVector.h>

namespace Ogre {
class Entity;
}  // namespace Ogre

namespace autoviz {
namespace rendering {

/** rviz_rendering::MeshShape — manual triangle mesh builder. */
class OgreMeshShape {
 public:
  explicit OgreMeshShape(Ogre::SceneManager* scene_manager,
                         Ogre::SceneNode* parent_node = nullptr);
  ~OgreMeshShape();

  void estimateVertexCount(size_t vcount);
  void beginTriangles();
  void addVertex(const Ogre::Vector3& position);
  void addVertex(const Ogre::Vector3& position, const Ogre::Vector3& normal);
  void addVertex(const Ogre::Vector3& position, const Ogre::Vector3& normal,
                 const Ogre::ColourValue& color);
  void addNormal(const Ogre::Vector3& normal);
  void addColor(const Ogre::ColourValue& color);
  void addTriangle(unsigned int p1, unsigned int p2, unsigned int p3);
  void endTriangles();
  void clear();
  Ogre::ManualObject* manualObject() { return manual_object_; }
  Ogre::Entity* entity() { return entity_; }
  Ogre::SceneNode* rootNode() { return scene_node_; }

 private:
  Ogre::SceneManager* scene_manager_ = nullptr;
  Ogre::SceneNode* scene_node_ = nullptr;
  Ogre::SceneNode* offset_node_ = nullptr;
  Ogre::Entity* entity_ = nullptr;
  Ogre::ManualObject* manual_object_ = nullptr;
  Ogre::MaterialPtr material_;
  std::string material_name_;
  bool started_ = false;
};

}  // namespace rendering
}  // namespace autoviz

#endif
