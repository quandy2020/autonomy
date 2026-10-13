/******************************************************************************
 * Copyright 2008, Willow Garage, Inc. · Copyright 2017, Bosch Software Innovations GmbH.
 * Adapted for Autoviz (BSD-3-Clause).
 *****************************************************************************/

/**
 * @file ogre_billboard_line.hpp
 * @brief Camera-facing line strip via Ogre BillboardChain.
 *
 * rviz_rendering::BillboardLine — thick polylines for Path displays, effort
 * circles, torque rings, and tool overlays.
 *
 * @see OgreSceneHost::setDisplayBillboardStrip()
 * @see OgreWrenchVisual
 * @see OgreEffortVisual
 */

#pragma once

#include <cstdint>
#include <functional>
#include <vector>

#include <OgreBillboardChain.h>
#include <OgreColourValue.h>
#include <OgreMaterial.h>
#include <OgreSharedPtr.h>
#include <OgreQuaternion.h>
#include <OgreVector3.h>

#include "autoviz/rendering/objects/ogre_object.hpp"

namespace Ogre {
class SceneManager;
class SceneNode;
class Any;
}  // namespace Ogre

namespace autoviz {
namespace rendering {

/**
 * @class OgreBillboardLine
 * @brief rviz_rendering::BillboardLine — camera-facing line strip.
 *
 * Internally manages one or more @c Ogre::BillboardChain containers when the
 * point count exceeds per-chain limits.
 */
class OgreBillboardLine : public OgreObject {
 public:
  /**
   * @brief Creates an empty billboard line under @p parent_node.
   *
   * @param scene_manager Non-null scene manager.
   * @param parent_node Parent scene node.
   */
  OgreBillboardLine(Ogre::SceneManager* scene_manager,
                    Ogre::SceneNode* parent_node = nullptr);

  /** @brief Destroys chains, material, and scene node. */
  ~OgreBillboardLine() override;

  /**
   * @brief Removes all points / chain elements.
   */
  void clear();

  /**
   * @brief Appends a point to the current line using the current color/width.
   * @param point World/parent-space position.
   */
  void addPoint(const Ogre::Vector3& point);

  /**
   * @brief Appends a point with an explicit color to the current line.
   * @param point Parent-space position.
   * @param color Vertex color (also drives alpha blending).
   */
  void addPoint(const Ogre::Vector3& point, const Ogre::ColourValue& color);

  /** @brief Finishes the current line; subsequent points start the next line. */
  void finishLine();

  /** @brief Sets the maximum number of points per line. */
  void setMaxPointsPerLine(uint32_t max);

  /** @brief Sets the number of lines. */
  void setNumLines(uint32_t num);

  void setPosition(const Ogre::Vector3& position) override;
  void setOrientation(const Ogre::Quaternion& orientation) override;
  /** @brief No-op (scale is meaningless for billboard chains, as in RViz). */
  void setScale(const Ogre::Vector3& scale) override;
  /** @brief Sets the strip colour from components. */
  void setColor(float r, float g, float b, float a) override;
  const Ogre::Vector3& getPosition() override;
  const Ogre::Quaternion& getOrientation() override;
  /** @brief No-op: there are no objects that can carry user data. */
  void setUserData(const Ogre::Any& data) override;

  /** @brief RViz-style alias of @ref sceneNode(). */
  Ogre::SceneNode* getSceneNode() { return scene_node_; }

  /** @brief Returns the shared line material. */
  Ogre::MaterialPtr getMaterial() { return material_; }

  /** @brief Returns chain containers (exposed for testing). */
  std::vector<Ogre::BillboardChain*> getChains() { return chain_containers_; }

  /**
   * @brief Sets the billboard strip width.
   * @param width Width in meters.
   */
  void setLineWidth(float width);

  /**
   * @brief Sets the strip colour (applied to existing elements).
   * @param color RGBA colour.
   */
  void setColor(const Ogre::ColourValue& color);

  /**
   * @brief Replace contents with a single polyline (@c num_lines = 1).
   *
   * @param points Ordered polyline vertices.
   * @param color Strip colour.
   * @param width Strip width.
   */
  void setPolyline(const std::vector<Ogre::Vector3>& points,
                   const Ogre::ColourValue& color, float width);

  /**
   * @brief Returns the owned scene node.
   * @return Scene node pointer.
   */
  Ogre::SceneNode* sceneNode() const { return scene_node_; }

 private:
  void setupChainContainers();
  Ogre::BillboardChain* createChain();
  void setupChainsInChainContainers() const;
  void incrementChainContainerIfNecessary();
  void changeAllElements(
      std::function<Ogre::BillboardChain::Element(Ogre::BillboardChain::Element)>
          change_element);

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

