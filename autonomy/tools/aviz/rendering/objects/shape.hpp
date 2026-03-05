/******************************************************************************
 * Copyright 2025 The Openbot Authors (duyongquan)
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *****************************************************************************/

#pragma once

#include <OGRE/OgreMaterial.h>
#include <OGRE/OgreSharedPtr.h>
#include <OGRE/OgreVector3.h>

#include <string>

#include "autonomy/tools/aviz/rendering/objects/object.hpp"
#include "autonomy/tools/aviz/rendering/visibility_control.hpp"

namespace Ogre {
class SceneManager;
class SceneNode;
class Any;
class Entity;
}  // namespace Ogre

namespace aviz {
namespace rendering {

class Shape : public Object {
 public:
  enum Type {
    Cone,
    Cube,
    Cylinder,
    Sphere,
    Mesh,
  };

  /**
   * @brief Constructor
   * @param shape_type Type of shape to create
   * @param scene_manager The Ogre scene manager this object is associated with
   * @param parent_node A scene node to use as the parent of this object. If NULL, uses the root scene node.
   */
  AVIZ_RENDERING_PUBLIC
  Shape(Type shape_type, Ogre::SceneManager* scene_manager, Ogre::SceneNode* parent_node = nullptr);

  AVIZ_RENDERING_PUBLIC
  virtual ~Shape();

  AVIZ_RENDERING_PUBLIC
  Type getType() { return type_; }

  /**
   * @brief Set the offset for this shape
   * @param offset Amount to offset the center of the object from the pivot point
   */
  AVIZ_RENDERING_PUBLIC
  void setOffset(const Ogre::Vector3& offset);

  AVIZ_RENDERING_PUBLIC
  virtual void setColor(float r, float g, float b, float a);

  AVIZ_RENDERING_PUBLIC
  void setColor(const Ogre::ColourValue& c);

  AVIZ_RENDERING_PUBLIC
  virtual void setPosition(const Ogre::Vector3& position);

  AVIZ_RENDERING_PUBLIC
  virtual void setOrientation(const Ogre::Quaternion& orientation);

  AVIZ_RENDERING_PUBLIC
  virtual void setScale(const Ogre::Vector3& scale);

  AVIZ_RENDERING_PUBLIC
  virtual const Ogre::Vector3& getPosition();

  AVIZ_RENDERING_PUBLIC
  virtual const Ogre::Quaternion& getOrientation();

  /**
   * @brief Get the root scene node (pivot node) for this object
   * @return The root scene node of this object
   */
  AVIZ_RENDERING_PUBLIC
  Ogre::SceneNode* getRootNode() { return scene_node_; }

  /**
   * @brief Sets user data on all ogre objects we own
   */
  AVIZ_RENDERING_PUBLIC
  void setUserData(const Ogre::Any& data);

  AVIZ_RENDERING_PUBLIC
  Ogre::Entity* getEntity() { return entity_; }

  AVIZ_RENDERING_PUBLIC
  Ogre::MaterialPtr getMaterial() { return material_; }

  AVIZ_RENDERING_PUBLIC
  static Ogre::Entity* createEntity(const std::string& name, Type shape_type, Ogre::SceneManager* scene_manager);

 protected:
  Ogre::SceneNode* scene_node_;
  Ogre::SceneNode* offset_node_;
  Ogre::Entity* entity_;
  Ogre::MaterialPtr material_;
  std::string material_name_;

  Type type_;
};

}  // namespace rendering
}  // namespace aviz
