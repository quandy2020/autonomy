/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

#pragma once

#ifdef AUTOVIZ_USE_OGRE

#include "autoviz/rendering/render_settings.hpp"

namespace Ogre {
class ManualObject;
class SceneManager;
class SceneNode;
}

namespace autoviz {
namespace rendering {

/** rviz_rendering::Axes — RGB axis lines at the origin. */
class OgreAxes {
 public:
  OgreAxes(Ogre::SceneManager* scene_manager, Ogre::SceneNode* parent_node);
  ~OgreAxes();

  void setLength(float length);
  void setVisible(bool visible);

 private:
  void rebuild();

  Ogre::SceneManager* scene_manager_ = nullptr;
  Ogre::SceneNode* scene_node_ = nullptr;
  Ogre::ManualObject* manual_object_ = nullptr;
  float length_ = 2.f;
  bool visible_ = true;
  bool dirty_ = true;
};

}  // namespace rendering
}  // namespace autoviz

#endif
