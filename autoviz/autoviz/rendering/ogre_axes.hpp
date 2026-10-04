/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

/**
 * @file ogre_axes.hpp
 * @brief Ogre RGB axis triad at the origin (rviz_rendering::Axes).
 *
 * Draws X=red, Y=green, Z=blue line segments for orientation cues, typically
 * alongside @ref OgreGrid when @ref ReferenceGridSettings::show_axes is set.
 *
 * @see OgreGrid
 * @see ReferenceGridSettings
 */

#pragma once

#include "autoviz/rendering/render_settings.hpp"

namespace Ogre {
class ManualObject;
class SceneManager;
class SceneNode;
}

namespace autoviz {
namespace rendering {

/**
 * @class OgreAxes
 * @brief rviz_rendering::Axes — RGB axis lines at the origin.
 *
 * @note Non-owning of @c SceneManager / parent node; owns its ManualObject.
 */
class OgreAxes {
 public:
  /**
   * @brief Creates axes under @p parent_node.
   *
   * @param scene_manager Non-null Ogre scene manager.
   * @param parent_node Parent scene node for the axes.
   */
  OgreAxes(Ogre::SceneManager* scene_manager, Ogre::SceneNode* parent_node);

  /** @brief Destroys ManualObject and scene node. */
  ~OgreAxes();

  /**
   * @brief Sets axis length in meters and marks geometry dirty.
   * @param length Length of each RGB segment from the origin.
   */
  void setLength(float length);

  /**
   * @brief Shows or hides the axes.
   * @param visible Scene-node visibility flag.
   */
  void setVisible(bool visible);

 private:
  /** @brief Rebuilds ManualObject from @c length_. */
  void rebuild();

  Ogre::SceneManager* scene_manager_ = nullptr; /**< Non-owning. */
  Ogre::SceneNode* scene_node_ = nullptr;       /**< Owned child node. */
  Ogre::ManualObject* manual_object_ = nullptr; /**< Owned line geometry. */
  float length_ = 1.f;                          /**< Axis length (m). */
  bool visible_ = true;                         /**< Scene-node visibility. */
  bool dirty_ = true;                           /**< Needs @ref rebuild(). */
};

}  // namespace rendering
}  // namespace autoviz

