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

#include <OGRE/OgreVector3.h>

// Local visibility macro (previously from visibility_control.hpp)
#ifndef AVIZ_RENDERING_PUBLIC
#define AVIZ_RENDERING_PUBLIC
#endif

namespace Ogre {
class SceneManager;
class SceneNode;
class Quaternion;
class Any;
}  // namespace Ogre

namespace aviz {
namespace rendering {

/**
 * @brief Base class for visible objects
 * Base object for scene entities
 *
 * Provides a minimal generic interface for rendering objects in the scene using
 * Ogre
 */
class AVIZ_RENDERING_PUBLIC Object
{
public:
    /**
     * @brief Constructor
     * @param scene_manager The Ogre scene manager this object should be part of
     */
    explicit Object(Ogre::SceneManager* scene_manager);
    virtual ~Object() = default;

    /**
     * @brief Set the position of this object
     * @param position Position vector to set to
     */
    virtual void setPosition(const Ogre::Vector3& position) = 0;

    /**
     * @brief Set the orientation of the object
     * @param orientation Quaternion orientation to set to
     */
    virtual void setOrientation(const Ogre::Quaternion& orientation) = 0;

    /**
     * @brief Set the scale of the object
     * @param scale Scale vector to set to
     */
    virtual void setScale(const Ogre::Vector3& scale) = 0;

    /**
     * @brief Set the color of the object. Values are in the range [0, 1]
     * @param r Red component
     * @param g Green component
     * @param b Blue component
     * @param a Alpha component
     */
    virtual void setColor(float r, float g, float b, float a) = 0;

    /**
     * @brief Get the local position of this object
     * @return The position
     */
    virtual const Ogre::Vector3& getPosition() = 0;

    /**
     * @brief Get the local orientation of this object
     * @return The orientation
     */
    virtual const Ogre::Quaternion& getOrientation() = 0;

    /**
     * @brief Set the user data on this object
     * @param data User data to set
     */
    virtual void setUserData(const Ogre::Any& data) = 0;

protected:
    Ogre::SceneManager*
        scene_manager_;  ///< Ogre scene manager this object is part of
};

}  // namespace rendering
}  // namespace aviz
