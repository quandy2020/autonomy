/******************************************************************************
 * Copyright 2023, Open Source Robotics Foundation, Inc.
 * Adapted for Autoviz (BSD-3-Clause).
 *****************************************************************************/

/**
 * @file ogre_screw_visual.hpp
 * @brief Linear/angular screw visualization (rviz_rendering::ScrewVisual).
 *
 * Draws linear and angular screw arrows with an angular circle, analogous to
 * @ref OgreWrenchVisual. Driven by @ref OgreSceneHost::setDisplayScrew().
 *
 * @see OgreWrenchVisual
 * @see OgreArrow
 * @see OgreBillboardLine
 */

#pragma once

#include <memory>

#include <OgreQuaternion.h>
#include <OgreVector3.h>

namespace Ogre {
class SceneManager;
class SceneNode;
}  // namespace Ogre

namespace autoviz {
namespace rendering {

class OgreArrow;
class OgreBillboardLine;

/**
 * @class OgreScrewVisual
 * @brief rviz_rendering::ScrewVisual — linear/angular screw visualization.
 */
class OgreScrewVisual {
 public:
  /**
   * @brief Creates linear/angular children under @p parent_node.
   *
   * @param scene_manager Non-null scene manager.
   * @param parent_node Parent scene node.
   */
  OgreScrewVisual(Ogre::SceneManager* scene_manager, Ogre::SceneNode* parent_node);

  /** @brief Destroys arrows, circle, and frame nodes. */
  ~OgreScrewVisual();

  /**
   * @brief Sets linear and angular screw vectors (frame-local).
   * @param linear Linear component.
   * @param angular Angular component.
   */
  void setScrew(const Ogre::Vector3& linear, const Ogre::Vector3& angular);

  /**
   * @brief Sets the frame origin position.
   * @param position World/parent position.
   */
  void setFramePosition(const Ogre::Vector3& position);

  /**
   * @brief Sets the frame orientation.
   * @param orientation Frame rotation.
   */
  void setFrameOrientation(const Ogre::Quaternion& orientation);

  /**
   * @brief Sets linear arrow RGBA.
   */
  void setLinearColor(float r, float g, float b, float a);

  /**
   * @brief Sets angular circle / arrow RGBA.
   */
  void setAngularColor(float r, float g, float b, float a);

  /**
   * @brief Scales the linear arrow.
   * @param scale Linear scale factor.
   */
  void setLinearScale(float scale);

  /**
   * @brief Scales the angular visualization.
   * @param scale Angular scale factor.
   */
  void setAngularScale(float scale);

  /**
   * @brief Sets arrow / circle line width.
   * @param width Width in meters.
   */
  void setWidth(float width);

  /**
   * @brief When @c true, hides near-zero linear/angular components.
   * @param hide Hide-small-values flag.
   */
  void setHideSmallValues(bool hide);

  /**
   * @brief Shows or hides the whole visual.
   * @param visible Visibility flag.
   */
  void setVisible(bool visible);

 private:
  std::unique_ptr<OgreArrow> arrow_linear_;
  std::unique_ptr<OgreArrow> arrow_angular_;
  std::unique_ptr<OgreBillboardLine> circle_angular_;
  std::unique_ptr<OgreArrow> circle_arrow_angular_;
  float linear_scale_ = 0.f;
  float angular_scale_ = 0.f;
  float width_ = 0.f;
  bool hide_small_values_ = true;
  Ogre::SceneNode* frame_node_ = nullptr;
  Ogre::SceneNode* linear_node_ = nullptr;
  Ogre::SceneNode* angular_node_ = nullptr;
  Ogre::SceneManager* scene_manager_ = nullptr;
};

}  // namespace rendering
}  // namespace autoviz

