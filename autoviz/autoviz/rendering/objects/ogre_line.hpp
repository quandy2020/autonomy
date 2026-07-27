/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

#pragma once

#ifdef AUTOVIZ_USE_OGRE

#include <OgreColourValue.h>
#include <OgreMaterial.h>
#include <OgreSharedPtr.h>
#include <OgreVector.h>

namespace Ogre {
class Any;
class ManualObject;
class SceneManager;
class SceneNode;
}  // namespace Ogre

namespace autoviz {
namespace rendering {

/** rviz_rendering::Line subset — wireframe segment via ManualObject. */
class OgreLine {
 public:
  explicit OgreLine(Ogre::SceneManager* scene_manager,
                    Ogre::SceneNode* parent_node = nullptr);
  ~OgreLine();

  void setPoints(Ogre::Vector3 start, Ogre::Vector3 end);
  void setVisible(bool visible);
  void setPosition(const Ogre::Vector3& position);
  void setOrientation(const Ogre::Quaternion& orientation);
  void setScale(const Ogre::Vector3& scale);
  void setColor(float r, float g, float b, float a);
  void setColor(const Ogre::ColourValue& color);
  const Ogre::Vector3& position() const;
  const Ogre::Quaternion& orientation() const;
  void setUserData(const Ogre::Any& data);

 private:
  Ogre::SceneManager* scene_manager_ = nullptr;
  Ogre::SceneNode* scene_node_ = nullptr;
  Ogre::ManualObject* manual_object_ = nullptr;
  Ogre::MaterialPtr material_;
  Ogre::ColourValue line_color_{1.f, 1.f, 1.f, 1.f};
};

}  // namespace rendering
}  // namespace autoviz

#endif
