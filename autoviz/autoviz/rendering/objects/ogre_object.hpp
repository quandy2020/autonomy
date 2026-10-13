/******************************************************************************
 * Copyright 2008, Willow Garage, Inc. · Copyright 2017, Open Source Robotics
 * Foundation, Inc. · Adapted for Autoviz (BSD-3-Clause).
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *   * Redistributions of source code must retain the above copyright notice,
 *     this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above copyright notice,
 *     this list of conditions and the following disclaimer in the documentation
 *     and/or other materials provided with the distribution.
 *   * Neither the name of the copyright holder nor the names of its
 *     contributors may be used to endorse or promote products derived from this
 *     software without specific prior written permission.
 * THIS SOFTWARE IS PROVIDED "AS IS" WITHOUT WARRANTY OF ANY KIND; see the
 * RViz BSD-3-Clause license for the full disclaimer.
 *****************************************************************************/

/**
 * @file ogre_object.hpp
 * @brief Abstract base class for Ogre-backed scene objects.
 *
 * Port of @c rviz_rendering::Object. Provides a uniform interface for setting
 * pose, scale, color and user data on shapes, arrows, lines, axes, etc.
 *
 * @see OgreShape
 * @see OgreArrow
 * @see OgreLine
 */

#pragma once

#include <OgreQuaternion.h>
#include <OgreVector.h>

namespace Ogre {
class SceneManager;
class SceneNode;
class Any;
}  // namespace Ogre

namespace autoviz {
namespace rendering {

/**
 * @class OgreObject
 * @brief Abstract base for objects that live in an Ogre scene.
 */
class OgreObject {
 public:
  /**
   * @brief Constructs the object bound to a scene manager.
   * @param scene_manager The scene manager this object should be part of.
   */
  explicit OgreObject(Ogre::SceneManager* scene_manager);
  virtual ~OgreObject();

  /** @brief Sets the position of this object. */
  virtual void setPosition(const Ogre::Vector3& position) = 0;

  /** @brief Sets the orientation of this object. */
  virtual void setOrientation(const Ogre::Quaternion& orientation) = 0;

  /**
   * @brief Sets the scale of the object, relative to its identity orientation.
   */
  virtual void setScale(const Ogre::Vector3& scale) = 0;

  /**
   * @brief Sets the color of the object. Values are in the range [0, 1].
   */
  virtual void setColor(float r, float g, float b, float a) = 0;

  /** @brief Returns the local position of this object. */
  virtual const Ogre::Vector3& getPosition() = 0;

  /** @brief Returns the local orientation of this object. */
  virtual const Ogre::Quaternion& getOrientation() = 0;

  /** @brief Sets user data on this object. */
  virtual void setUserData(const Ogre::Any& data) = 0;

 protected:
  Ogre::SceneManager* scene_manager_;  ///< Ogre scene manager this object is part of.
};

}  // namespace rendering
}  // namespace autoviz
