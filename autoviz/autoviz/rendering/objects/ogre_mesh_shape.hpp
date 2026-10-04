/******************************************************************************
 * Copyright 2008, Willow Garage, Inc.
 * Adapted for Autoviz (BSD-3-Clause).
 *****************************************************************************/

/**
 * @file ogre_mesh_shape.hpp
 * @brief Manual triangle mesh builder (rviz_rendering::MeshShape).
 *
 * Incremental API: @ref beginTriangles() → @ref addVertex() /
 * @ref addTriangle() → @ref endTriangles(). Used when uploading dynamic
 * triangle meshes without going through MeshManager.
 *
 * @see OgreShape
 * @see OgreSceneHost::setDisplayMeshes()
 */

#pragma once

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

/**
 * @class OgreMeshShape
 * @brief rviz_rendering::MeshShape — manual triangle mesh builder.
 *
 * After @ref endTriangles(), geometry may be exposed as ManualObject and/or
 * converted Entity depending on implementation path.
 */
class OgreMeshShape {
 public:
  /**
   * @brief Creates an empty mesh shape under an optional parent.
   *
   * @param scene_manager Non-null scene manager.
   * @param parent_node Parent node; @c nullptr uses the root.
   */
  explicit OgreMeshShape(Ogre::SceneManager* scene_manager,
                         Ogre::SceneNode* parent_node = nullptr);

  /** @brief Clears geometry and destroys owned Ogre objects. */
  ~OgreMeshShape();

  /**
   * @brief Hints ManualObject about upcoming vertex count.
   * @param vcount Expected number of vertices.
   */
  void estimateVertexCount(size_t vcount);

  /**
   * @brief Begins a triangle-list ManualObject section.
   * @note Must pair with @ref endTriangles().
   */
  void beginTriangles();

  /**
   * @brief Appends a vertex with position only.
   * @param position Vertex position.
   */
  void addVertex(const Ogre::Vector3& position);

  /**
   * @brief Appends a vertex with position and normal.
   * @param position Vertex position.
   * @param normal Vertex normal.
   */
  void addVertex(const Ogre::Vector3& position, const Ogre::Vector3& normal);

  /**
   * @brief Appends a vertex with position, normal, and color.
   * @param position Vertex position.
   * @param normal Vertex normal.
   * @param color Vertex colour.
   */
  void addVertex(const Ogre::Vector3& position, const Ogre::Vector3& normal,
                 const Ogre::ColourValue& color);

  /**
   * @brief Sets the normal for the next / current vertex attribute stream.
   * @param normal Vertex normal.
   */
  void addNormal(const Ogre::Vector3& normal);

  /**
   * @brief Sets the color for the next / current vertex attribute stream.
   * @param color Vertex colour.
   */
  void addColor(const Ogre::ColourValue& color);

  /**
   * @brief Adds an indexed triangle (vertex indices from @ref addVertex()).
   * @param p1 First index.
   * @param p2 Second index.
   * @param p3 Third index.
   */
  void addTriangle(unsigned int p1, unsigned int p2, unsigned int p3);

  /**
   * @brief Ends the ManualObject section and finalizes geometry.
   */
  void endTriangles();

  /**
   * @brief Removes all geometry and resets builder state.
   */
  void clear();

  /**
   * @brief Returns the underlying ManualObject (may be null before begin).
   * @return ManualObject pointer.
   */
  Ogre::ManualObject* manualObject() { return manual_object_; }

  /**
   * @brief Returns an Entity view of the mesh if created.
   * @return Entity pointer or @c nullptr.
   */
  Ogre::Entity* entity() { return entity_; }

  /**
   * @brief Returns the root scene node.
   * @return Scene node pointer.
   */
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

