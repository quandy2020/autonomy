/******************************************************************************
 * Copyright 2023, Open Source Robotics Foundation, Inc.
 * Adapted for Autoviz (BSD-3-Clause).
 *****************************************************************************/

#pragma once

#ifdef AUTOVIZ_USE_OGRE

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

/** rviz_rendering::ScrewVisual — linear/angular screw visualization. */
class OgreScrewVisual {
 public:
  OgreScrewVisual(Ogre::SceneManager* scene_manager, Ogre::SceneNode* parent_node);
  ~OgreScrewVisual();

  void setScrew(const Ogre::Vector3& linear, const Ogre::Vector3& angular);
  void setFramePosition(const Ogre::Vector3& position);
  void setFrameOrientation(const Ogre::Quaternion& orientation);
  void setLinearColor(float r, float g, float b, float a);
  void setAngularColor(float r, float g, float b, float a);
  void setLinearScale(float scale);
  void setAngularScale(float scale);
  void setWidth(float width);
  void setHideSmallValues(bool hide);
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

#endif
