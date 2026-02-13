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

#include <cstdint>
#include <functional>
#include <vector>

#include <OGRE/OgreBillboardChain.h>
#include <OGRE/OgreColourValue.h>
#include <OGRE/OgreMaterial.h>
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
class BillboardChain;
}  // namespace Ogre

namespace aviz {
namespace rendering {

/**
 * @brief Billboard line rendering object
 * Similar to rviz_rendering::BillboardLine
 *
 * Displays a multi-segment line strip rendered as billboards (always facing camera)
 */
class AVIZ_RENDERING_PUBLIC BillboardLine : public Object
{
public:
    /**
     * @brief Constructor
     * @param scene_manager The Ogre scene manager this object is part of
     * @param parent_node A scene node to use as the parent of this object. If NULL, uses the root scene node.
     */
    AVIZ_RENDERING_PUBLIC
    explicit BillboardLine(Ogre::SceneManager* scene_manager, Ogre::SceneNode* parent_node = nullptr);
    AVIZ_RENDERING_PUBLIC
    ~BillboardLine() override;

    AVIZ_RENDERING_PUBLIC
    void clear();

    AVIZ_RENDERING_PUBLIC
    void finishLine();

    AVIZ_RENDERING_PUBLIC
    void addPoint(const Ogre::Vector3& point);

    AVIZ_RENDERING_PUBLIC
    void addPoint(const Ogre::Vector3& point, const Ogre::ColourValue& color);

    AVIZ_RENDERING_PUBLIC
    void setLineWidth(float width);

    AVIZ_RENDERING_PUBLIC
    void setMaxPointsPerLine(uint32_t max);

    AVIZ_RENDERING_PUBLIC
    void setNumLines(uint32_t num);

    // overrides from Object
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
     * @brief We have no objects that we can set user data on
     */
    void setUserData(const Ogre::Any& data) override {
        (void)data;
    }

    Ogre::MaterialPtr getMaterial() {
        return material_;
    }

    typedef std::vector<Ogre::BillboardChain*> V_ChainContainers;
    /// exposed for testing
    V_ChainContainers getChains() {
        return chain_containers_;
    }

private:
    void setupChainContainers();
    Ogre::BillboardChain* createChain();
    void changeAllElements(std::function<Ogre::BillboardChain::Element(Ogre::BillboardChain::Element)> change_element);
    void incrementChainContainerIfNecessary();
    void setupChainsInChainContainers() const;

    Ogre::SceneNode* scene_node_;

    V_ChainContainers chain_containers_;
    Ogre::MaterialPtr material_;

    Ogre::ColourValue color_;
    float width_;

    uint32_t num_lines_;
    uint32_t max_points_per_line_;
    uint32_t chains_per_container_;

    uint32_t current_line_;
    uint32_t current_chain_container_;
    uint32_t elements_in_current_chain_container_;
};

}  // namespace rendering
}  // namespace aviz
