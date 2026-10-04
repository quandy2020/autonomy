/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

/**
 * @file ogre_line.hpp
 * @brief Wireframe segment via ManualObject (rviz_rendering::Line subset).
 *
 * Used by Measure tool overlays and other simple two-point wires under
 * @ref OgreSceneHost.
 *
 * @see OgreSceneHost::setToolLineSegment()
 * @see OgreArrow
 */

#pragma once

#include <OgreColourValue.h>
#include <OgreMaterial.h>
#include <OgreSharedPtr.h>
#include <OgreVector.h>

namespace Ogre {
class Any;
class ManualObject;
class SceneManager;
class SceneNode;
}  // namespace Ogre

namespace autoviz {
namespace rendering {

/**
 * @class OgreLine
 * @brief rviz_rendering::Line subset — wireframe segment via ManualObject.
 *
 * Owns a child scene node, ManualObject, and material. Endpoints are set in
 * the node's local space via @ref setPoints().
 */
class OgreLine {
 public:
  /**
   * @brief Creates the line under an optional parent node.
   *
   * @param scene_manager Non-null scene manager.
   * @param parent_node Parent node; @c nullptr attaches to the root.
   */
  explicit OgreLine(Ogre::SceneManager* scene_manager,
                    Ogre::SceneNode* parent_node = nullptr);

  /** @brief Destroys ManualObject, material, and scene node. */
  ~OgreLine();

  /**
   * @brief Sets the segment endpoints in local space.
   * @param start Start point.
   * @param end End point.
   */
  void setPoints(Ogre::Vector3 start, Ogre::Vector3 end);

  /**
   * @brief Shows or hides the line.
   * @param visible Visibility flag.
   */
  void setVisible(bool visible);

  /**
   * @brief Sets the scene-node position.
   * @param position World/parent-relative translation.
   */
  void setPosition(const Ogre::Vector3& position);

  /**
   * @brief Sets the scene-node orientation.
   * @param orientation Quaternion rotation.
   */
  void setOrientation(const Ogre::Quaternion& orientation);

  /**
   * @brief Sets the scene-node scale.
   * @param scale Non-uniform scale.
   */
  void setScale(const Ogre::Vector3& scale);

  /**
   * @brief Sets RGBA color components.
   * @param r Red \[0,1\].
   * @param g Green \[0,1\].
   * @param b Blue \[0,1\].
   * @param a Alpha \[0,1\].
   */
  void setColor(float r, float g, float b, float a);

  /**
   * @brief Sets color from an Ogre colour value.
   * @param color RGBA colour.
   */
  void setColor(const Ogre::ColourValue& color);

  /**
   * @brief Returns the scene-node position.
   * @return Const reference to position.
   */
  const Ogre::Vector3& position() const;

  /**
   * @brief Returns the scene-node orientation.
   * @return Const reference to orientation.
   */
  const Ogre::Quaternion& orientation() const;

  /**
   * @brief Attaches arbitrary user data to the movable.
   * @param data Ogre Any payload (e.g. pick identity).
   */
  void setUserData(const Ogre::Any& data);

 private:
  Ogre::SceneManager* scene_manager_ = nullptr;
  Ogre::SceneNode* scene_node_ = nullptr;
  Ogre::ManualObject* manual_object_ = nullptr;
  Ogre::MaterialPtr material_;
  Ogre::ColourValue line_color_{1.f, 1.f, 1.f, 1.f};
};

}  // namespace rendering
}  // namespace autoviz

