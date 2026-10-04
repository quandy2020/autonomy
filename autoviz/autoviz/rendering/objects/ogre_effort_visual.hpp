/******************************************************************************
 * Copyright 2023, Open Source Robotics Foundation, Inc.
 * Adapted for Autoviz (BSD-3-Clause).
 *****************************************************************************/

/**
 * @file ogre_effort_visual.hpp
 * @brief Per-joint effort visualization (rviz_rendering::EffortVisual).
 *
 * For each enabled joint, draws a colour-coded effort circle
 * (@ref OgreBillboardLine) and an optional magnitude arrow (@ref OgreArrow)
 * in the joint frame. Colour is mapped through @ref getRainbowColor().
 *
 * Typically driven by EffortDisplay via the Ogre scene host.
 *
 * @see OgreArrow
 * @see OgreBillboardLine
 * @see OgreWrenchVisual
 */

#pragma once

#include <map>
#include <memory>
#include <string>

#include <OgreColourValue.h>
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
 * @class OgreEffortVisual
 * @brief rviz_rendering::EffortVisual — joint effort circles and arrows.
 *
 * Call @ref setFramePosition() / @ref setFrameOrientation() to place each
 * joint frame, @ref setFrameEnabled() to show/hide, and @ref setEffort() to
 * update magnitude relative to @p max_effort.
 */
class OgreEffortVisual {
 public:
  /**
   * @brief Creates an empty visual under @p parent_node.
   *
   * @param scene_manager Non-null Ogre scene manager.
   * @param parent_node Parent scene node for joint frame children.
   * @param width Initial circle / arrow line width.
   * @param scale Initial geometric scale factor.
   */
  OgreEffortVisual(Ogre::SceneManager* scene_manager, Ogre::SceneNode* parent_node,
                   float width, float scale);

  /**
   * @brief Maps a normalized value in roughly \\[0,1\\] to a rainbow RGBA colour.
   *
   * @param value Scalar used for hue selection (typically |effort|/max_effort).
   * @param[out] color Filled with the resulting colour.
   */
  void getRainbowColor(float value, Ogre::ColourValue& color);

  /**
   * @brief Updates the drawn effort for @p joint_name.
   *
   * @param joint_name Joint key (must already have a frame via setFrame*).
   * @param effort Signed effort value.
   * @param max_effort Denominator for colour / scale normalization.
   */
  void setEffort(const std::string& joint_name, double effort, double max_effort);

  /**
   * @brief Sets the world/parent position of a joint frame.
   *
   * @param joint_name Joint key.
   * @param position Frame origin.
   */
  void setFramePosition(const std::string& joint_name, const Ogre::Vector3& position);

  /**
   * @brief Sets the orientation of a joint frame.
   *
   * @param joint_name Joint key.
   * @param orientation Frame rotation.
   */
  void setFrameOrientation(const std::string& joint_name,
                           const Ogre::Quaternion& orientation);

  /**
   * @brief Enables or disables drawing for a joint.
   *
   * @param joint_name Joint key.
   * @param enabled When @c false, circle/arrow for that joint are hidden.
   */
  void setFrameEnabled(const std::string& joint_name, bool enabled);

  /**
   * @brief Sets circle / arrow line width for subsequent updates.
   * @param width Width in meters.
   */
  void setWidth(float width);

  /**
   * @brief Sets overall geometric scale for subsequent updates.
   * @param scale Scale factor.
   */
  void setScale(float scale);

 private:
  /** Per-joint effort circle billboards. */
  std::map<std::string, std::unique_ptr<OgreBillboardLine>> effort_circle_;
  /** Per-joint effort direction arrows. */
  std::map<std::string, std::unique_ptr<OgreArrow>> effort_arrow_;
  /** Per-joint enable flags. */
  std::map<std::string, bool> effort_enabled_;
  /** Cached joint frame positions. */
  std::map<std::string, Ogre::Vector3> position_;
  /** Cached joint frame orientations. */
  std::map<std::string, Ogre::Quaternion> orientation_;
  Ogre::SceneManager* scene_manager_ = nullptr; /**< Non-owning. */
  Ogre::SceneNode* parent_node_ = nullptr;      /**< Non-owning. */
  float width_ = 0.f;  /**< Current line width. */
  float scale_ = 0.f;  /**< Current scale. */
};

}  // namespace rendering
}  // namespace autoviz

