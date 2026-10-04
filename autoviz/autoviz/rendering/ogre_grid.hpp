/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

/**
 * @file ogre_grid.hpp
 * @brief Ogre ManualObject ground grid (rviz_rendering::Grid Lines style).
 *
 * Draws an XY-plane reference grid (REP-103 Z-up) under a parent scene node.
 * Settings match @ref ReferenceGridSettings used by the OpenGL @ref GridRenderer.
 *
 * @see ReferenceGridSettings
 * @see OgreAxes
 * @see GridRenderer
 */

#pragma once

#include <cstdint>

#include <OgreColourValue.h>

#include "autoviz/rendering/render_settings.hpp"

namespace Ogre {
class ManualObject;
class SceneManager;
class SceneNode;
}

namespace autoviz {
namespace rendering {

/**
 * @class OgreGrid
 * @brief rviz_rendering::Grid (Lines style) — XY plane reference grid.
 *
 * Owns a child @c SceneNode and @c ManualObject. Call @ref setSettings() then
 * visibility; geometry rebuilds lazily when dirty.
 *
 * @note Non-copyable. Does not own @p scene_manager or the parent node.
 */
class OgreGrid {
 public:
  /**
   * @brief Creates the grid under @p parent_node.
   *
   * @param scene_manager Non-null Ogre scene manager.
   * @param parent_node Parent for the grid's scene node (typically scene root).
   */
  OgreGrid(Ogre::SceneManager* scene_manager, Ogre::SceneNode* parent_node);

  /** @brief Destroys ManualObject and scene node. */
  ~OgreGrid();

  OgreGrid(const OgreGrid&) = delete;
  OgreGrid& operator=(const OgreGrid&) = delete;

  /**
   * @brief Applies grid settings and marks geometry dirty for rebuild.
   *
   * @param settings Cell count, length, color, axes options.
   * @see ReferenceGridSettings
   */
  void setSettings(const ReferenceGridSettings& settings);

  /**
   * @brief Shows or hides the grid scene node.
   * @param visible When @c false, the ManualObject is not rendered.
   */
  void setVisible(bool visible);

  /**
   * @brief Returns the grid's owned scene node.
   * @return Non-null scene node after construction.
   */
  Ogre::SceneNode* sceneNode() const { return scene_node_; }

 private:
  /** @brief Rebuilds ManualObject line geometry from @c settings_. */
  void rebuild();

  Ogre::SceneManager* scene_manager_ = nullptr; /**< Non-owning. */
  Ogre::SceneNode* scene_node_ = nullptr;       /**< Owned child node. */
  Ogre::ManualObject* manual_object_ = nullptr; /**< Owned line geometry. */
  ReferenceGridSettings settings_;              /**< Last applied settings. */
  bool visible_ = true;                         /**< Scene-node visibility. */
  bool dirty_ = true;                           /**< Needs @ref rebuild(). */
};

}  // namespace rendering
}  // namespace autoviz

