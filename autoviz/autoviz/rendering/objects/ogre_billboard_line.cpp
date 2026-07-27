/******************************************************************************
 * Copyright 2008, Willow Garage, Inc. · Copyright 2017, Bosch Software Innovations GmbH.
 * Adapted for Autoviz (BSD-3-Clause).
 *****************************************************************************/

#include "autoviz/rendering/objects/ogre_billboard_line.hpp"

#ifdef AUTOVIZ_USE_OGRE

#include <sstream>
#include <stdexcept>
#include <string>

#include <OgreBillboardChain.h>
#include <OgreMaterialManager.h>
#include <OgreSceneManager.h>
#include <OgreSceneNode.h>
#include <OgreTechnique.h>

#include "autoviz/rendering/ogre_material_manager.hpp"

namespace autoviz {
namespace rendering {
namespace {

constexpr uint32_t kMaxElements = (65536 / 4);

}  // namespace

OgreBillboardLine::OgreBillboardLine(Ogre::SceneManager* scene_manager,
                                     Ogre::SceneNode* parent_node)
    : scene_manager_(scene_manager) {
  if (scene_manager_ == nullptr) {
    throw std::invalid_argument("OgreBillboardLine requires SceneManager");
  }
  if (parent_node == nullptr) {
    parent_node = scene_manager_->getRootSceneNode();
  }
  scene_node_ = parent_node->createChildSceneNode();

  static int count = 0;
  const std::string material_name = "AvizBillboardLineMaterial" + std::to_string(count++);
  material_ = OgreMaterialManager::createMaterialWithNoLighting(material_name);

  setNumLines(1);
  setMaxPointsPerLine(max_points_per_line_);
}

OgreBillboardLine::~OgreBillboardLine() {
  if (scene_manager_ == nullptr) {
    return;
  }
  for (Ogre::BillboardChain* chain : chain_containers_) {
    scene_manager_->destroyBillboardChain(chain);
  }
  if (scene_node_ != nullptr) {
    scene_manager_->destroySceneNode(scene_node_);
    scene_node_ = nullptr;
  }
  if (!material_.isNull()) {
    material_->unload();
    Ogre::MaterialManager::getSingleton().remove(material_->getName(), "AvizOgre");
  }
}

void OgreBillboardLine::clear() {
  for (Ogre::BillboardChain* chain : chain_containers_) {
    chain->clearAllChains();
  }
  current_line_ = 0;
  current_chain_container_ = 0;
  elements_in_current_chain_container_ = 0;
}

Ogre::BillboardChain* OgreBillboardLine::createChain() {
  std::stringstream ss;
  static int count = 0;
  ss << "AvizBillboardLineChain" << count++;
  Ogre::BillboardChain* chain = scene_manager_->createBillboardChain(ss.str());
  chain->setMaterialName(material_->getName(), "AvizOgre");
  scene_node_->attachObject(chain);
  chain_containers_.push_back(chain);
  return chain;
}

void OgreBillboardLine::setupChainContainers() {
  const uint32_t total_points = max_points_per_line_ * num_lines_;
  uint32_t num_chains = total_points / kMaxElements;
  if (total_points % kMaxElements != 0) {
    ++num_chains;
  }
  for (uint32_t i = static_cast<uint32_t>(chain_containers_.size()); i < num_chains; ++i) {
    createChain();
  }
  chains_per_container_ =
      max_points_per_line_ > 0 ? kMaxElements / max_points_per_line_ : 1;
  if (max_points_per_line_ > kMaxElements) {
    chains_per_container_ = 1;
  }
  setupChainsInChainContainers();
}

void OgreBillboardLine::setupChainsInChainContainers() const {
  auto it = chain_containers_.begin();
  const auto end = chain_containers_.end();
  for (; it != end; ++it) {
    (*it)->setMaxChainElements(max_points_per_line_);
    if (it + 1 == end) {
      const uint32_t lines_left = num_lines_ % chains_per_container_;
      (*it)->setNumberOfChains((lines_left == 0) ? chains_per_container_ : lines_left);
    } else {
      (*it)->setNumberOfChains(chains_per_container_);
    }
  }
}

void OgreBillboardLine::setMaxPointsPerLine(uint32_t max) {
  max_points_per_line_ = std::max(2u, max);
  setupChainContainers();
}

void OgreBillboardLine::setNumLines(uint32_t num) {
  num_lines_ = std::max(1u, num);
  setupChainContainers();
}

void OgreBillboardLine::incrementChainContainerIfNecessary() {
  ++elements_in_current_chain_container_;
  if (elements_in_current_chain_container_ > kMaxElements) {
    ++current_chain_container_;
    elements_in_current_chain_container_ = 1;
  }
}

void OgreBillboardLine::setLineWidth(float width) {
  width_ = width;
  changeAllElements([width](Ogre::BillboardChain::Element element) {
    element.width = width;
    return element;
  });
}

void OgreBillboardLine::setColor(const Ogre::ColourValue& color) {
  OgreMaterialManager::enableAlphaBlending(material_, color.a);
  color_ = color;
  changeAllElements([color](Ogre::BillboardChain::Element element) {
    element.colour = color;
    return element;
  });
}

void OgreBillboardLine::setColor(float r, float g, float b, float a) {
  setColor(Ogre::ColourValue(r, g, b, a));
}

void OgreBillboardLine::addPoint(const Ogre::Vector3& point) {
  incrementChainContainerIfNecessary();
  Ogre::BillboardChain::Element element;
  element.position = point;
  element.width = width_;
  element.colour = color_;
  chain_containers_[current_chain_container_]->addChainElement(
      current_line_ % chains_per_container_, element);
}

void OgreBillboardLine::changeAllElements(
    std::function<Ogre::BillboardChain::Element(Ogre::BillboardChain::Element)>
        change_element) {
  for (uint32_t line = 0; line < num_lines_; ++line) {
    Ogre::BillboardChain* container =
        chain_containers_[line / chains_per_container_];
    const uint32_t chain_index = line % chains_per_container_;
    const size_t elements_in_chain = container->getNumChainElements(chain_index);
    for (uint32_t i = 0; i < elements_in_chain; ++i) {
      Ogre::BillboardChain::Element element =
          container->getChainElement(chain_index, i);
      container->updateChainElement(chain_index, i, change_element(element));
    }
  }
}

void OgreBillboardLine::setPolyline(const std::vector<Ogre::Vector3>& points,
                                    const Ogre::ColourValue& color, float width) {
  if (points.empty()) {
    clear();
    return;
  }
  setNumLines(1);
  setMaxPointsPerLine(static_cast<uint32_t>(points.size()));
  clear();
  setLineWidth(width);
  setColor(color);

  current_line_ = 0;
  for (const Ogre::Vector3& point : points) {
    incrementChainContainerIfNecessary();
    Ogre::BillboardChain::Element element;
    element.position = point;
    element.width = width_;
    element.colour = color_;
    chain_containers_[current_chain_container_]->addChainElement(
        current_line_ % chains_per_container_, element);
  }
}

}  // namespace rendering
}  // namespace autoviz

#endif
