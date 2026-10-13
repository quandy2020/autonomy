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
 * @file ogre_axes.hpp
 * @brief Cylinder-based X/Y/Z axes object (X=red, Y=green, Z=blue).
 *
 * Port of @c rviz_rendering::Axes. Not to be confused with
 * @ref OgreReferenceAxes (line-based viewport reference triad).
 *
 * @see OgreShape
 * @see OgreReferenceAxes
 */

#pragma once

#include <memory>

#include <OgreColourValue.h>
#include <OgreQuaternion.h>
#include <OgreVector.h>

#include "autoviz/rendering/objects/ogre_object.hpp"
#include "autoviz/rendering/objects/ogre_shape.hpp"

namespace Ogre {
class SceneManager;
class SceneNode;
class Any;
}  // namespace Ogre

namespace autoviz {
namespace rendering {

/**
 * @class OgreAxes
 * @brief An object that displays a set of X/Y/Z axes (X=Red, Y=Green, Z=Blue).
 */
class OgreAxes : public OgreObject {
 public:
  /**
   * @param manager Scene manager this object is a part of.
   * @param parent_node Parent scene node; @c nullptr uses the root node.
   * @param length Length of the axes.
   * @param radius Radius of the axes.
   */
  explicit OgreAxes(Ogre::SceneManager* manager,
                    Ogre::SceneNode* parent_node = nullptr, float length = 1.0f,
                    float radius = 0.1f);
  ~OgreAxes() override;

  OgreAxes(const OgreAxes&) = delete;
  OgreAxes& operator=(const OgreAxes&) = delete;

  /** @brief Sets length and radius of the axes (also resets default colors). */
  void set(float length, float radius);

  void setOrientation(const Ogre::Quaternion& orientation) override;
  void setPosition(const Ogre::Vector3& position) override;
  void setScale(const Ogre::Vector3& scale) override;

  /** @brief No-op: the axes have several colors (see @ref setXColor etc.). */
  void setColor(float r, float g, float b, float a) override;

  const Ogre::Vector3& getPosition() override;
  const Ogre::Quaternion& getOrientation() override;

  /** @brief Returns the scene node of this object. */
  Ogre::SceneNode* getSceneNode() { return scene_node_; }

  /** @brief Sets user data on all Ogre objects owned by the axes. */
  void setUserData(const Ogre::Any& data) override;

  OgreShape& getXShape() { return *x_axis_; }
  OgreShape& getYShape() { return *y_axis_; }
  OgreShape& getZShape() { return *z_axis_; }

  void setXColor(const Ogre::ColourValue& col);
  void setYColor(const Ogre::ColourValue& col);
  void setZColor(const Ogre::ColourValue& col);
  void setToDefaultColors();

  static const Ogre::ColourValue& getDefaultXColor();
  static const Ogre::ColourValue& getDefaultYColor();
  static const Ogre::ColourValue& getDefaultZColor();

 private:
  Ogre::SceneNode* scene_node_ = nullptr;

  std::unique_ptr<OgreShape> x_axis_;  ///< Cylinder for the X-axis.
  std::unique_ptr<OgreShape> y_axis_;  ///< Cylinder for the Y-axis.
  std::unique_ptr<OgreShape> z_axis_;  ///< Cylinder for the Z-axis.

  static const Ogre::ColourValue default_x_color_;
  static const Ogre::ColourValue default_y_color_;
  static const Ogre::ColourValue default_z_color_;
};

}  // namespace rendering
}  // namespace autoviz
