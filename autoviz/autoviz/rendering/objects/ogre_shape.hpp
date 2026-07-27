/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

#pragma once

#ifdef AUTOVIZ_USE_OGRE

#include <string>

#include <OgreColourValue.h>
#include <OgreMaterial.h>
#include <OgreSharedPtr.h>
#include <OgreVector.h>

namespace Ogre {
class Any;
class Entity;
class SceneManager;
class SceneNode;
}  // namespace Ogre

namespace autoviz {
namespace rendering {

/** rviz_rendering::Shape subset — Entity + rviz_*.mesh primitives. */
class OgreShape {
 public:
  enum Type { kCone, kCube, kCylinder, kSphere, kCapsule };

  OgreShape(Type shape_type, Ogre::SceneManager* scene_manager,
            Ogre::SceneNode* parent_node = nullptr);
  ~OgreShape();

  Type type() const { return type_; }

  void setOffset(const Ogre::Vector3& offset);
  void setColor(float r, float g, float b, float a);
  void setColor(const Ogre::ColourValue& color);
  void setPosition(const Ogre::Vector3& position);
  void setOrientation(const Ogre::Quaternion& orientation);
  void setScale(const Ogre::Vector3& scale);
  const Ogre::Vector3& position() const;
  const Ogre::Quaternion& orientation() const;
  Ogre::SceneNode* rootNode() { return scene_node_; }
  Ogre::Entity* entity() { return entity_; }
  Ogre::MaterialPtr material() { return material_; }
  void setUserData(const Ogre::Any& data);

  static Ogre::Entity* createEntity(const std::string& name, Type shape_type,
                                    Ogre::SceneManager* scene_manager);

 private:
  Ogre::SceneManager* scene_manager_ = nullptr;
  Ogre::SceneNode* scene_node_ = nullptr;
  Ogre::SceneNode* offset_node_ = nullptr;
  Ogre::Entity* entity_ = nullptr;
  Ogre::MaterialPtr material_;
  std::string material_name_;
  Type type_ = kCube;
};

}  // namespace rendering
}  // namespace autoviz

#endif
