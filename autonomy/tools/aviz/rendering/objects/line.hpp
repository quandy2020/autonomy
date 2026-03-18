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
#include <OGRE/OgreSceneNode.h>
#include <OGRE/OgreSharedPtr.h>
#include <OGRE/OgreVector3.h>

#include "autonomy/tools/aviz/rendering/objects/object.hpp"

// Local visibility macro (previously from visibility_control.hpp)
#ifndef AVIZ_RENDERING_PUBLIC
#define AVIZ_RENDERING_PUBLIC
#endif

namespace Ogre {
class SceneManager;
class SceneNode;
class Quaternion;
class Any;
class ColourValue;
class ManualObject;
}  // namespace Ogre

namespace aviz {
namespace rendering {

/**
 * @brief Line rendering object
 * Line visualization object
 *
 * Represents a straight wireframe line between two points
 */
class AVIZ_RENDERING_PUBLIC Line : public Object {
 public:
  /**
   * @brief Constructor
   * @param scene_manager The Ogre scene manager this object is part of
   * @param parent_node A scene node to use as the parent of this object. Uses the root scene node if null.
   */
  AVIZ_RENDERING_PUBLIC
  explicit Line(Ogre::SceneManager* scene_manager, Ogre::SceneNode* parent_node = nullptr);

  AVIZ_RENDERING_PUBLIC
  virtual ~Line();

  /**
   * @brief Set the start and end point of the line
   * @param start The start point
   * @param end The end point
   */
  AVIZ_RENDERING_PUBLIC
  void setPoints(Ogre::Vector3 start, Ogre::Vector3 end);

  AVIZ_RENDERING_PUBLIC
  void setVisible(bool visible);

  AVIZ_RENDERING_PUBLIC
  void setPosition(const Ogre::Vector3& position) override;

  AVIZ_RENDERING_PUBLIC
  void setOrientation(const Ogre::Quaternion& orientation) override;

  AVIZ_RENDERING_PUBLIC
  void setScale(const Ogre::Vector3& scale) override;

  AVIZ_RENDERING_PUBLIC
  void setColor(float r, float g, float b, float a) override;

  /**
   * @brief Set the color of the object using Ogre colour definitions
   */
  AVIZ_RENDERING_PUBLIC
  virtual void setColor(const Ogre::ColourValue& c);

  AVIZ_RENDERING_PUBLIC
  const Ogre::Vector3& getPosition() override;

  AVIZ_RENDERING_PUBLIC
  const Ogre::Quaternion& getOrientation() override;

  AVIZ_RENDERING_PUBLIC
  void setUserData(const Ogre::Any& data) override;

 protected:
  Ogre::SceneNode* scene_node_;
  Ogre::ManualObject* manual_object_;
  Ogre::MaterialPtr manual_object_material_;
};

}  // namespace rendering
}  // namespace aviz
