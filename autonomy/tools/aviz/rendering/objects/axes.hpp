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

#include <cstddef>
#include <cstdint>
#include <memory>

#include <OGRE/OgreVector3.h>

#include "autonomy/tools/aviz/rendering/objects/object.hpp"
#include "autonomy/tools/aviz/rendering/objects/shape.hpp"

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
}  // namespace Ogre

namespace aviz {
namespace rendering {

/**
 * @brief Axes rendering object
 * Similar to rviz_rendering::Axes
 *
 * Displays a set of X/Y/Z axes, with X=Red, Y=Green, Z=Blue
 */
class AVIZ_RENDERING_PUBLIC Axes : public Object
{
public:
    /**
     * @brief Constructor
     * @param scene_manager The Ogre scene manager this object is part of
     * @param parent_node A scene node to use as the parent of this object. If NULL, uses the root scene node.
     * @param length Length of the axes
     * @param radius Radius of the axes
     */
    AVIZ_RENDERING_PUBLIC
    explicit Axes(Ogre::SceneManager* scene_manager, Ogre::SceneNode* parent_node = nullptr, float length = 1.0f,
                  float radius = 0.1f);
    ~Axes() override;

    /**
     * @brief Set the parameters on this object
     * @param length Length of the axes
     * @param radius Radius of the axes
     */
    AVIZ_RENDERING_PUBLIC
    void set(float length, float radius);

    AVIZ_RENDERING_PUBLIC
    void setOrientation(const Ogre::Quaternion& orientation) override;

    AVIZ_RENDERING_PUBLIC
    void setPosition(const Ogre::Vector3& position) override;

    AVIZ_RENDERING_PUBLIC
    void setScale(const Ogre::Vector3& scale) override;

    AVIZ_RENDERING_PUBLIC
    void setColor(float r, float g, float b, float a) override;

    AVIZ_RENDERING_PUBLIC
    const Ogre::Vector3& getPosition() override;

    AVIZ_RENDERING_PUBLIC
    const Ogre::Quaternion& getOrientation() override;

    /**
     * @brief Get the scene node associated with this object
     */
    Ogre::SceneNode* getSceneNode() {
        return scene_node_;
    }

    /**
     * @brief Sets user data on all ogre objects we own
     */
    AVIZ_RENDERING_PUBLIC
    void setUserData(const Ogre::Any& data) override;

    AVIZ_RENDERING_PUBLIC
    Shape& getXShape() {
        return *x_axis_;
    }

    AVIZ_RENDERING_PUBLIC
    Shape& getYShape() {
        return *y_axis_;
    }

    AVIZ_RENDERING_PUBLIC
    Shape& getZShape() {
        return *z_axis_;
    }

    AVIZ_RENDERING_PUBLIC
    void setXColor(const Ogre::ColourValue& col);

    AVIZ_RENDERING_PUBLIC
    void setYColor(const Ogre::ColourValue& col);

    AVIZ_RENDERING_PUBLIC
    void setZColor(const Ogre::ColourValue& col);

    AVIZ_RENDERING_PUBLIC
    void setToDefaultColors();

    AVIZ_RENDERING_PUBLIC
    static const Ogre::ColourValue& getDefaultXColor();

    AVIZ_RENDERING_PUBLIC
    static const Ogre::ColourValue& getDefaultYColor();

    AVIZ_RENDERING_PUBLIC
    static const Ogre::ColourValue& getDefaultZColor();

private:
    // prohibit copying
    Axes(const Axes& other) : Object(nullptr) {
        (void)other;
    }
    Axes& operator=(const Axes& other) {
        (void)other;
        return *this;
    }

    Ogre::SceneNode* scene_node_;

    std::unique_ptr<Shape> x_axis_;  ///< Cylinder for the X-axis
    std::unique_ptr<Shape> y_axis_;  ///< Cylinder for the Y-axis
    std::unique_ptr<Shape> z_axis_;  ///< Cylinder for the Z-axis

    static const Ogre::ColourValue default_x_color_;
    static const Ogre::ColourValue default_y_color_;
    static const Ogre::ColourValue default_z_color_;
};

}  // namespace rendering
}  // namespace aviz
