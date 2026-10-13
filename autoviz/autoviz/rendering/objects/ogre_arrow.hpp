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
#include <OgreQuaternion.h>
#include <OgreVector.h>

#include "autoviz/rendering/objects/ogre_object.hpp"

namespace Ogre {
class Any;
class SceneManager;
class SceneNode;
}  // namespace Ogre

namespace autoviz {
namespace rendering {

class OgreShape;

/**
 * @class OgreArrow
 * @brief rviz_rendering::Arrow — cylinder shaft + cone head.
 *
 * Faithful port of @c rviz_rendering::Arrow: the arrow points along -Z at
 * identity orientation. @ref setOrientation() and @ref setDirection() change
 * the direction the arrow points.
 */
class OgreArrow : public OgreObject {
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
  ~OgreArrow() override;

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
  void setColor(float r, float g, float b, float a) override;

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

  /** @brief Sets only the cone head color (RGBA components). */
  void setHeadColor(float r, float g, float b, float a = 1.0f);

  /**
   * @brief Sets only the cylinder shaft color.
   * @param color Shaft colour.
   */
  void setShaftColor(const Ogre::ColourValue& color);

  /** @brief Sets only the cylinder shaft color (RGBA components). */
  void setShaftColor(float r, float g, float b, float a = 1.0f);

  /**
   * @brief Sets the root orientation.
   * @param orientation Rotation quaternion.
   */
  void setOrientation(const Ogre::Quaternion& orientation) override;

  /**
   * @brief Sets the root position.
   * @param position Translation.
   */
  void setPosition(const Ogre::Vector3& position) override;

  /**
   * @brief Orients the arrow to point along @p direction.
   * @param direction Desired axis in parent space; a zero vector is ignored.
   */
  void setDirection(const Ogre::Vector3& direction);

  /**
   * @brief Sets the root scale.
   * @param scale Non-uniform scale.
   */
  void setScale(const Ogre::Vector3& scale) override;

  /** @brief Returns root position (OgreObject interface). */
  const Ogre::Vector3& getPosition() override;

  /** @brief Returns root orientation (OgreObject interface). */
  const Ogre::Quaternion& getOrientation() override;

  /** @brief Returns root position. */
  const Ogre::Vector3& position() const;

  /** @brief Returns root orientation. */
  const Ogre::Quaternion& orientation() const;

  /**
   * @brief Returns the root scene node.
   * @return Scene node pointer.
   */
  Ogre::SceneNode* sceneNode() { return scene_node_; }

  /** @brief RViz-style alias of @ref sceneNode(). */
  Ogre::SceneNode* getSceneNode() { return scene_node_; }

  /**
   * @brief Attaches user data to both shapes.
   * @param data Ogre Any payload.
   */
  void setUserData(const Ogre::Any& data) override;

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

  /** @brief RViz-style alias of @ref shaft(). */
  OgreShape* getShaft() { return shaft_; }

  /** @brief RViz-style alias of @ref head(). */
  OgreShape* getHead() { return head_; }

 private:
  Ogre::SceneNode* scene_node_ = nullptr;
  OgreShape* shaft_ = nullptr;
  OgreShape* head_ = nullptr;
};

}  // namespace rendering
}  // namespace autoviz

