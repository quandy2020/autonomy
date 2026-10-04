/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

/**
 * @file ogre_arrow.hpp
 * @brief Arrow composed of cylinder shaft + cone head Entities.
 *
 * rviz_rendering::Arrow subset used by tools (Pose, Measure), wrench/screw
 * visuals, and display arrows via @ref OgreSceneHost.
 *
 * @see OgreShape
 * @see OgreWrenchVisual
 * @see OgreScrewVisual
 */

#pragma once

#include <OgreColourValue.h>
#include <OgreVector.h>

namespace Ogre {
class Any;
class SceneManager;
class SceneNode;
class Quaternion;
}  // namespace Ogre

namespace autoviz {
namespace rendering {

class OgreShape;

/**
 * @class OgreArrow
 * @brief rviz_rendering::Arrow subset — cylinder shaft + cone head.
 *
 * Default orientation points along +X (rviz convention). Use
 * @ref setDirection() to aim the arrow at a vector.
 */
class OgreArrow {
 public:
  /**
   * @brief Creates shaft and head shapes under an optional parent.
   *
   * @param scene_manager Non-null scene manager.
   * @param parent_node Parent node; @c nullptr uses the root.
   * @param shaft_length Shaft length along +X.
   * @param shaft_diameter Shaft diameter.
   * @param head_length Cone head length.
   * @param head_diameter Cone head base diameter.
   */
  OgreArrow(Ogre::SceneManager* scene_manager,
            Ogre::SceneNode* parent_node = nullptr, float shaft_length = 1.f,
            float shaft_diameter = 0.1f, float head_length = 0.3f,
            float head_diameter = 0.2f);

  /** @brief Destroys shaft, head, and scene node. */
  ~OgreArrow();

  /**
   * @brief Resizes shaft and head dimensions.
   *
   * @param shaft_length Shaft length.
   * @param shaft_diameter Shaft diameter.
   * @param head_length Head length.
   * @param head_diameter Head diameter.
   */
  void set(float shaft_length, float shaft_diameter, float head_length,
           float head_diameter);

  /**
   * @brief Sets a uniform color on shaft and head.
   */
  void setColor(float r, float g, float b, float a);

  /**
   * @brief Sets a uniform colour on shaft and head.
   * @param color RGBA colour.
   */
  void setColor(const Ogre::ColourValue& color);

  /**
   * @brief Sets only the cone head color.
   * @param color Head colour.
   */
  void setHeadColor(const Ogre::ColourValue& color);

  /**
   * @brief Sets only the cylinder shaft color.
   * @param color Shaft colour.
   */
  void setShaftColor(const Ogre::ColourValue& color);

  /**
   * @brief Sets the root orientation.
   * @param orientation Rotation quaternion.
   */
  void setOrientation(const Ogre::Quaternion& orientation);

  /**
   * @brief Sets the root position.
   * @param position Translation.
   */
  void setPosition(const Ogre::Vector3& position);

  /**
   * @brief Orients the arrow to point along @p direction (normalized internally).
   * @param direction Desired axis in parent space.
   */
  void setDirection(const Ogre::Vector3& direction);

  /**
   * @brief Sets the root scale.
   * @param scale Non-uniform scale.
   */
  void setScale(const Ogre::Vector3& scale);

  /** @brief Returns root position. */
  const Ogre::Vector3& position() const;

  /** @brief Returns root orientation. */
  const Ogre::Quaternion& orientation() const;

  /**
   * @brief Returns the root scene node.
   * @return Scene node pointer.
   */
  Ogre::SceneNode* sceneNode() { return scene_node_; }

  /**
   * @brief Attaches user data to both shapes.
   * @param data Ogre Any payload.
   */
  void setUserData(const Ogre::Any& data);

  /**
   * @brief Returns the shaft shape.
   * @return Owned @ref OgreShape (cylinder).
   */
  OgreShape* shaft() { return shaft_; }

  /**
   * @brief Returns the head shape.
   * @return Owned @ref OgreShape (cone).
   */
  OgreShape* head() { return head_; }

 private:
  Ogre::SceneManager* scene_manager_ = nullptr;
  Ogre::SceneNode* scene_node_ = nullptr;
  OgreShape* shaft_ = nullptr;
  OgreShape* head_ = nullptr;
};

}  // namespace rendering
}  // namespace autoviz

