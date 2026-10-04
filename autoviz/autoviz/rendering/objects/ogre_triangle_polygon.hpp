/******************************************************************************
 * Copyright 2024, Open Source Robotics Foundation, Inc.
 * Adapted for Autoviz (BSD-3-Clause).
 *****************************************************************************/

/**
 * @file ogre_triangle_polygon.hpp
 * @brief Single textured triangle ManualObject (rviz_rendering::TrianglePolygon).
 *
 * Builds one triangle (upper or lower half of a quad) with optional vertex
 * color — used by grid / map style overlays that tessellate cells.
 *
 * @see OgreMeshShape
 */

#pragma once

#include <string>

#include <OgreColourValue.h>
#include <OgreManualObject.h>
#include <OgreSceneManager.h>
#include <OgreSceneNode.h>
#include <OgreVector.h>

namespace autoviz {
namespace rendering {

/**
 * @class OgreTrianglePolygon
 * @brief rviz_rendering::TrianglePolygon — single textured triangle ManualObject.
 *
 * Constructed fully formed; geometry is immutable after construction.
 */
class OgreTrianglePolygon {
 public:
  /**
   * @brief Creates a triangle ManualObject attached to @p node.
   *
   * @param manager Non-null scene manager.
   * @param node Parent scene node that receives the ManualObject.
   * @param O First triangle vertex.
   * @param A Second triangle vertex.
   * @param B Third triangle vertex.
   * @param name Unique ManualObject name.
   * @param color Vertex / material colour.
   * @param use_color When @c true, apply @p color; else use texture only.
   * @param upper_triangle Selects UV winding for upper vs lower quad half.
   */
  OgreTrianglePolygon(Ogre::SceneManager* manager, Ogre::SceneNode* node,
                      const Ogre::Vector3& O, const Ogre::Vector3& A,
                      const Ogre::Vector3& B, const std::string& name,
                      const Ogre::ColourValue& color, bool use_color,
                      bool upper_triangle);

  /** @brief Destroys the ManualObject. */
  ~OgreTrianglePolygon();

  /**
   * @brief Returns the owned ManualObject.
   * @return ManualObject pointer.
   */
  Ogre::ManualObject* manualObject() { return manual_; }

 private:
  Ogre::ManualObject* manual_ = nullptr;
  Ogre::SceneManager* manager_ = nullptr;
};

}  // namespace rendering
}  // namespace autoviz

