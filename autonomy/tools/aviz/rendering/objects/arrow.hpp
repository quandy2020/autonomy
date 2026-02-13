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

#include "autonomy/tools/aviz/rendering/objects/object.hpp"

// Local visibility macro (previously from visibility_control.hpp)
#ifndef AVIZ_RENDERING_PUBLIC
#define AVIZ_RENDERING_PUBLIC
#endif

namespace Ogre {
class SceneManager;
class SceneNode;
class Quaternion;
class ColourValue;
class Any;
}  // namespace Ogre

namespace aviz {
namespace rendering {

class Shape;

/**
 * @brief Arrow rendering object
 * Similar to rviz_rendering::Arrow
 *
 * An arrow consisting of a cylinder (shaft) and a cone (head)
 * The base of the arrow is at the position sent to setPosition().
 * The arrow points in the direction of the negative Z axis by
 * default, and -Z is the identity direction of it.
 */
class AVIZ_RENDERING_PUBLIC Arrow : public Object
{
public:
    /**
     * @brief Constructor
     * @param scene_manager The Ogre scene manager to use
     * @param parent_node A scene node to use as the parent of this object. If NULL, uses the root scene node.
     * @param shaft_length Length of the arrow's shaft
     * @param shaft_diameter Diameter of the arrow's shaft
     * @param head_length Length of the arrow's head
     * @param head_diameter Diameter of the arrow's head
     */
    Arrow(Ogre::SceneManager* scene_manager, Ogre::SceneNode* parent_node = nullptr, float shaft_length = 1.0f,
          float shaft_diameter = 0.1f, float head_length = 0.3f, float head_diameter = 0.2f);
    virtual ~Arrow();

    /**
     * @brief Set the parameters for this arrow
     */
    void set(float shaft_length, float shaft_diameter, float head_length, float head_diameter);

    /**
     * @brief Set the color of this arrow. Sets both the head and shaft color to the same value.
     */
    virtual void setColor(float r, float g, float b, float a);
    void setColor(const Ogre::ColourValue& color);

    /**
     * @brief Set the color of the arrow's head
     */
    void setHeadColor(float r, float g, float b, float a = 1.0f);
    void setHeadColor(const Ogre::ColourValue& color);

    /**
     * @brief Set the color of the arrow's shaft
     */
    void setShaftColor(float r, float g, float b, float a = 1.0f);
    void setShaftColor(const Ogre::ColourValue& color);

    /** @brief Set the orientation. Note that negative Z is the identity orientation. */
    virtual void setOrientation(const Ogre::Quaternion& orientation);

    /** @brief Set the position of the base of the arrow */
    virtual void setPosition(const Ogre::Vector3& position);

    /** @brief Set the direction of the arrow */
    void setDirection(const Ogre::Vector3& direction);

    virtual void setScale(const Ogre::Vector3& scale);
    virtual const Ogre::Vector3& getPosition();
    virtual const Ogre::Quaternion& getOrientation();

    /**
     * @brief Get the scene node associated with this arrow
     */
    Ogre::SceneNode* getSceneNode() {
        return scene_node_;
    }

    /**
     * @brief Sets user data on all ogre objects we own
     */
    void setUserData(const Ogre::Any& data);

    Shape* getShaft() {
        return shaft_;
    }
    Shape* getHead() {
        return head_;
    }

private:
    Ogre::SceneNode* scene_node_;

    Shape* shaft_;  ///< Cylinder used for the shaft of the arrow
    Shape* head_;   ///< Cone used for the head of the arrow
};

}  // namespace rendering
}  // namespace aviz
