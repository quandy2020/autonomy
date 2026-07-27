/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

#pragma once

#ifdef AUTOVIZ_USE_OGRE

#include <OgreColourValue.h>
#include <OgreVector.h>

namespace Ogre {
class Any;
class SceneManager;
class SceneNode;
class Quaternion;
}  // namespace Ogre

namespace autoviz {
namespace rendering {

class OgreShape;

/** rviz_rendering::Arrow subset — cylinder shaft + cone head Entities. */
class OgreArrow {
 public:
  OgreArrow(Ogre::SceneManager* scene_manager,
            Ogre::SceneNode* parent_node = nullptr, float shaft_length = 1.f,
            float shaft_diameter = 0.1f, float head_length = 0.3f,
            float head_diameter = 0.2f);
  ~OgreArrow();

  void set(float shaft_length, float shaft_diameter, float head_length,
           float head_diameter);
  void setColor(float r, float g, float b, float a);
  void setColor(const Ogre::ColourValue& color);
  void setHeadColor(const Ogre::ColourValue& color);
  void setShaftColor(const Ogre::ColourValue& color);
  void setOrientation(const Ogre::Quaternion& orientation);
  void setPosition(const Ogre::Vector3& position);
  void setDirection(const Ogre::Vector3& direction);
  void setScale(const Ogre::Vector3& scale);
  const Ogre::Vector3& position() const;
  const Ogre::Quaternion& orientation() const;
  Ogre::SceneNode* sceneNode() { return scene_node_; }
  void setUserData(const Ogre::Any& data);
  OgreShape* shaft() { return shaft_; }
  OgreShape* head() { return head_; }

 private:
  Ogre::SceneManager* scene_manager_ = nullptr;
  Ogre::SceneNode* scene_node_ = nullptr;
  OgreShape* shaft_ = nullptr;
  OgreShape* head_ = nullptr;
};

}  // namespace rendering
}  // namespace autoviz

#endif
