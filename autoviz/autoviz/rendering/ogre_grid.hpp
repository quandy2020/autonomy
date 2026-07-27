/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

#pragma once

#ifdef AUTOVIZ_USE_OGRE

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

/** rviz_rendering::Grid (Lines style) — XY plane reference grid (REP-103 Z-up). */
class OgreGrid {
 public:
  OgreGrid(Ogre::SceneManager* scene_manager, Ogre::SceneNode* parent_node);
  ~OgreGrid();

  OgreGrid(const OgreGrid&) = delete;
  OgreGrid& operator=(const OgreGrid&) = delete;

  void setSettings(const ReferenceGridSettings& settings);
  void setVisible(bool visible);
  Ogre::SceneNode* sceneNode() const { return scene_node_; }

 private:
  void rebuild();

  Ogre::SceneManager* scene_manager_ = nullptr;
  Ogre::SceneNode* scene_node_ = nullptr;
  Ogre::ManualObject* manual_object_ = nullptr;
  ReferenceGridSettings settings_;
  bool visible_ = true;
  bool dirty_ = true;
};

}  // namespace rendering
}  // namespace autoviz

#endif
