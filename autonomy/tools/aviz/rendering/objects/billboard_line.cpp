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

#include "autonomy/tools/aviz/rendering/objects/billboard_line.hpp"

#include <cassert>
#include <iostream>
#include <sstream>
#include <string>

#include <OGRE/OgreBillboardChain.h>
#include <OGRE/OgreMaterialManager.h>
#include <OGRE/OgreQuaternion.h>
#include <OGRE/OgreSceneManager.h>
#include <OGRE/OgreSceneNode.h>
#include <OGRE/OgreTechnique.h>
#include <OGRE/OgreVector3.h>

#define ROS_PACKAGE_NAME "aviz_rendering"

static const uint32_t MAX_ELEMENTS = (65536 / 4);

namespace aviz {
namespace rendering {

BillboardLine::BillboardLine(Ogre::SceneManager* scene_manager, Ogre::SceneNode* parent_node)
    : Object(scene_manager),
      width_(0.1f),
      num_lines_(1),
      max_points_per_line_(100),
      chains_per_container_(0),
      current_line_(0),
      current_chain_container_(0),
      elements_in_current_chain_container_(0) {
    if (!parent_node) {
        parent_node = scene_manager_->getRootSceneNode();
    }

    scene_node_ = parent_node->createChildSceneNode();

    static int count = 0;
    std::string material_name = "BillboardLineMaterial" + std::to_string(count++);
    material_ = Ogre::MaterialManager::getSingleton().create(material_name, ROS_PACKAGE_NAME);
    material_->setReceiveShadows(false);
    Ogre::Technique* technique = material_->getTechnique(0);
    if (technique) {
        technique->setLightingEnabled(false);
    }

    setNumLines(num_lines_);
    setMaxPointsPerLine(max_points_per_line_);
}

BillboardLine::~BillboardLine() {
    for (auto& chain : chain_containers_) {
        scene_manager_->destroyBillboardChain(chain);
    }

    scene_manager_->destroySceneNode(scene_node_);

    material_->unload();
    Ogre::MaterialManager::getSingleton().remove(material_->getHandle());
}

void BillboardLine::clear() {
    for (auto& chain : chain_containers_) {
        chain->clearAllChains();
    }

    current_line_ = 0;
    current_chain_container_ = 0;
    elements_in_current_chain_container_ = 0;
}

void BillboardLine::setupChainContainers() {
    uint32_t total_points = max_points_per_line_ * num_lines_;
    uint32_t num_chains = total_points / MAX_ELEMENTS;
    if (total_points % MAX_ELEMENTS != 0) {
        ++num_chains;
    }

    for (uint32_t i = static_cast<uint32_t>(chain_containers_.size()); i < num_chains; ++i) {
        createChain();
    }

    chains_per_container_ = max_points_per_line_ > 0 ? MAX_ELEMENTS / max_points_per_line_ : 1;
    if (max_points_per_line_ > MAX_ELEMENTS) {
        chains_per_container_ = 1;
    }

    setupChainsInChainContainers();
}

Ogre::BillboardChain* BillboardLine::createChain() {
    std::stringstream ss;
    static int count = 0;
    ss << "BillboardLine chain" << count++;
    Ogre::BillboardChain* chain = scene_manager_->createBillboardChain(ss.str());
    chain->setMaterialName(material_->getName());
    scene_node_->attachObject(chain);

    chain_containers_.push_back(chain);

    return chain;
}

void BillboardLine::setupChainsInChainContainers() const {
    auto it = chain_containers_.begin();
    auto end = chain_containers_.end();
    for (; it != end; ++it) {
        (*it)->setMaxChainElements(max_points_per_line_);

        // shorten the number of chains in the last bbchain, to avoid memory wasteage
        if (it + 1 == end) {
            uint32_t lines_left = num_lines_ % chains_per_container_;

            // Handle the case where num_lines_ is a multiple of lines_per_chain
            (*it)->setNumberOfChains((lines_left == 0) ? chains_per_container_ : lines_left);
        } else {
            (*it)->setNumberOfChains(chains_per_container_);
        }
    }
}

void BillboardLine::setMaxPointsPerLine(uint32_t max) {
    max_points_per_line_ = max;

    setupChainContainers();
}

void BillboardLine::setNumLines(uint32_t num) {
    num_lines_ = num;

    setupChainContainers();
}

void BillboardLine::finishLine() {
    ++current_line_;

    assert(current_line_ <= num_lines_);
}

void BillboardLine::addPoint(const Ogre::Vector3& point) {
    addPoint(point, color_);
}

void BillboardLine::addPoint(const Ogre::Vector3& point, const Ogre::ColourValue& color) {
    assert(current_line_ < num_lines_);
    assert(chain_containers_[current_chain_container_]->getNumChainElements(current_line_ % chains_per_container_) <=
           max_points_per_line_);

    incrementChainContainerIfNecessary();

    // Enable alpha blending if needed
    if (color.a < 0.9998) {
        Ogre::Technique* technique = material_->getTechnique(0);
        if (technique) {
            Ogre::Pass* pass = technique->getPass(0);
            if (pass) {
                pass->setSceneBlending(Ogre::SBT_TRANSPARENT_ALPHA);
                pass->setDepthWriteEnabled(false);
            }
        }
    }

    Ogre::BillboardChain::Element e;
    e.position = point;
    e.width = width_;
    e.colour = color;
    chain_containers_[current_chain_container_]->addChainElement(current_line_ % chains_per_container_, e);
}

void BillboardLine::incrementChainContainerIfNecessary() {
    ++elements_in_current_chain_container_;
    if (elements_in_current_chain_container_ > MAX_ELEMENTS) {
        ++current_chain_container_;
        elements_in_current_chain_container_ = 1;
    }
}

void BillboardLine::setLineWidth(float width) {
    width_ = width;

    changeAllElements([width](Ogre::BillboardChain::Element element) {
        element.width = width;
        return element;
    });
}

void BillboardLine::setPosition(const Ogre::Vector3& position) {
    scene_node_->setPosition(position);
}

void BillboardLine::setOrientation(const Ogre::Quaternion& orientation) {
    scene_node_->setOrientation(orientation);
}

void BillboardLine::setScale(const Ogre::Vector3& scale) {
    // Setting scale doesn't really make sense here
    (void)scale;
}

void BillboardLine::setColor(float r, float g, float b, float a) {
    // Enable alpha blending if needed
    if (a < 0.9998) {
        Ogre::Technique* technique = material_->getTechnique(0);
        if (technique) {
            Ogre::Pass* pass = technique->getPass(0);
            if (pass) {
                pass->setSceneBlending(Ogre::SBT_TRANSPARENT_ALPHA);
                pass->setDepthWriteEnabled(false);
            }
        }
    }

    color_ = Ogre::ColourValue(r, g, b, a);

    changeAllElements([this](Ogre::BillboardChain::Element element) {
        element.colour = color_;
        return element;
    });
}

void BillboardLine::changeAllElements(
    std::function<Ogre::BillboardChain::Element(Ogre::BillboardChain::Element)> change_element) {
    for (uint32_t line = 0; line < num_lines_; ++line) {
        Ogre::BillboardChain* container = chain_containers_[line / chains_per_container_];
        uint32_t chain_index = line % chains_per_container_;
        size_t elements_in_chain = container->getNumChainElements(chain_index);

        for (uint32_t i = 0; i < elements_in_chain; ++i) {
            Ogre::BillboardChain::Element element = container->getChainElement(chain_index, i);
            Ogre::BillboardChain::Element new_element = change_element(element);
            container->updateChainElement(chain_index, i, new_element);
        }
    }
}

const Ogre::Vector3& BillboardLine::getPosition() {
    return scene_node_->getPosition();
}

const Ogre::Quaternion& BillboardLine::getOrientation() {
    return scene_node_->getOrientation();
}

}  // namespace rendering
}  // namespace aviz
