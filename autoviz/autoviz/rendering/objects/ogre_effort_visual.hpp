/******************************************************************************
 * Copyright 2023, Open Source Robotics Foundation, Inc.
 * Adapted for Autoviz (BSD-3-Clause).
 *****************************************************************************/

#pragma once

#ifdef AUTOVIZ_USE_OGRE

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

/** rviz_rendering::EffortVisual — joint effort circles. */
class OgreEffortVisual {
 public:
  OgreEffortVisual(Ogre::SceneManager* scene_manager, Ogre::SceneNode* parent_node,
                   float width, float scale);

  void getRainbowColor(float value, Ogre::ColourValue& color);
  void setEffort(const std::string& joint_name, double effort, double max_effort);
  void setFramePosition(const std::string& joint_name, const Ogre::Vector3& position);
  void setFrameOrientation(const std::string& joint_name, const Ogre::Quaternion& orientation);
  void setFrameEnabled(const std::string& joint_name, bool enabled);
  void setWidth(float width);
  void setScale(float scale);

 private:
  std::map<std::string, std::unique_ptr<OgreBillboardLine>> effort_circle_;
  std::map<std::string, std::unique_ptr<OgreArrow>> effort_arrow_;
  std::map<std::string, bool> effort_enabled_;
  std::map<std::string, Ogre::Vector3> position_;
  std::map<std::string, Ogre::Quaternion> orientation_;
  Ogre::SceneManager* scene_manager_ = nullptr;
  Ogre::SceneNode* parent_node_ = nullptr;
  float width_ = 0.f;
  float scale_ = 0.f;
};

}  // namespace rendering
}  // namespace autoviz

#endif
