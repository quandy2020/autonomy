/******************************************************************************
 * Copyright 2008, Willow Garage, Inc. · Copyright 2017, Bosch Software Innovations GmbH.
 * Adapted for Autoviz (BSD-3-Clause).
 *****************************************************************************/

#pragma once

#ifdef AUTOVIZ_USE_OGRE

#include <cstdint>
#include <functional>
#include <vector>

#include <OgreBillboardChain.h>
#include <OgreColourValue.h>
#include <OgreMaterial.h>
#include <OgreSharedPtr.h>
#include <OgreVector3.h>

namespace Ogre {
class SceneManager;
class SceneNode;
}  // namespace Ogre

namespace autoviz {
namespace rendering {

/** rviz_rendering::BillboardLine — camera-facing line strip (Path, thick polylines). */
class OgreBillboardLine {
 public:
  OgreBillboardLine(Ogre::SceneManager* scene_manager, Ogre::SceneNode* parent_node);
  ~OgreBillboardLine();

  void clear();
  void addPoint(const Ogre::Vector3& point);
  void setLineWidth(float width);
  void setColor(const Ogre::ColourValue& color);
  void setColor(float r, float g, float b, float a);
  /** Replace contents with a single polyline (num_lines = 1). */
  void setPolyline(const std::vector<Ogre::Vector3>& points,
                   const Ogre::ColourValue& color, float width);

  Ogre::SceneNode* sceneNode() const { return scene_node_; }

 private:
  void setMaxPointsPerLine(uint32_t max);
  void setNumLines(uint32_t num);
  void setupChainContainers();
  Ogre::BillboardChain* createChain();
  void setupChainsInChainContainers() const;
  void incrementChainContainerIfNecessary();
  void changeAllElements(
      std::function<Ogre::BillboardChain::Element(Ogre::BillboardChain::Element)>
          change_element);

  Ogre::SceneManager* scene_manager_ = nullptr;
  Ogre::SceneNode* scene_node_ = nullptr;
  std::vector<Ogre::BillboardChain*> chain_containers_;
  Ogre::MaterialPtr material_;
  Ogre::ColourValue color_;
  float width_ = 0.1f;
  uint32_t num_lines_ = 1;
  uint32_t max_points_per_line_ = 100;
  uint32_t chains_per_container_ = 0;
  uint32_t current_line_ = 0;
  uint32_t current_chain_container_ = 0;
  uint32_t elements_in_current_chain_container_ = 0;
};

}  // namespace rendering
}  // namespace autoviz

#endif
