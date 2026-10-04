/******************************************************************************
 * Copyright 2019, Martin Idel · Adapted for Autoviz (BSD-3-Clause).
 *****************************************************************************/

/**
 * @file ogre_wrench_visual.hpp
 * @brief Force/torque wrench visualization (rviz_rendering::WrenchVisual).
 *
 * Draws a force arrow plus a torque circle with direction arrow at a frame
 * pose. Driven by @ref OgreSceneHost::setDisplayWrench().
 *
 * @see OgreArrow
 * @see OgreBillboardLine
 * @see OgreScrewVisual
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
 * @class OgreWrenchVisual
 * @brief rviz_rendering::WrenchVisual — force/torque arrow visualization.
 *
 * Call @ref setWrench() then frame pose / scale / color setters. Visibility
 * hides the entire frame node.
 */
class OgreWrenchVisual {
 public:
  /**
   * @brief Creates force/torque children under @p parent_node.
   *
   * @param scene_manager Non-null scene manager.
   * @param parent_node Parent scene node.
   */
  OgreWrenchVisual(Ogre::SceneManager* scene_manager, Ogre::SceneNode* parent_node);

  /** @brief Destroys arrows, circle, and frame nodes. */
  ~OgreWrenchVisual();

  /**
   * @brief Sets force and torque vectors (frame-local).
   * @param force Force vector.
   * @param torque Torque vector.
   */
  void setWrench(const Ogre::Vector3& force, const Ogre::Vector3& torque);

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
   * @brief Sets force arrow RGBA.
   */
  void setForceColor(float r, float g, float b, float a);

  /**
   * @brief Sets torque circle / arrow RGBA.
   */
  void setTorqueColor(float r, float g, float b, float a);

  /**
   * @brief Scales the force arrow length.
   * @param scale Force scale factor.
   */
  void setForceScale(float scale);

  /**
   * @brief Scales the torque visualization.
   * @param scale Torque scale factor.
   */
  void setTorqueScale(float scale);

  /**
   * @brief Sets arrow / circle line width.
   * @param width Width in meters.
   */
  void setWidth(float width);

  /**
   * @brief Shows or hides the whole visual.
   * @param visible Visibility flag.
   */
  void setVisible(bool visible);

 private:
  void createTorqueDirectionCircle(const Ogre::Quaternion& orientation) const;
  void setTorqueDirectionArrow(const Ogre::Quaternion& orientation) const;
  Ogre::Quaternion getDirectionOfRotationRelativeToTorque(const Ogre::Vector3& torque,
                                                          const Ogre::Vector3& axis_z) const;
  void updateForceArrow() const;
  void updateTorque() const;

  std::unique_ptr<OgreArrow> arrow_force_;
  std::unique_ptr<OgreArrow> arrow_torque_;
  std::unique_ptr<OgreBillboardLine> circle_torque_;
  std::unique_ptr<OgreArrow> circle_arrow_torque_;
  Ogre::Vector3 force_arrow_direction_;
  Ogre::Vector3 torque_arrow_direction_;
  float force_scale_ = 1.f;
  float torque_scale_ = 1.f;
  float width_ = 1.f;
  Ogre::SceneNode* frame_node_ = nullptr;
  Ogre::SceneNode* force_node_ = nullptr;
  Ogre::SceneNode* torque_node_ = nullptr;
  Ogre::SceneManager* scene_manager_ = nullptr;
};

}  // namespace rendering
}  // namespace autoviz

