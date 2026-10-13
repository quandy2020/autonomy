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
 * @file ogre_grid.hpp
 * @brief RViz-style grid of cells drawn with lines or billboards.
 *
 * Port of @c rviz_rendering::Grid. A grid with an identity orientation is
 * drawn along the XZ plane (as in RViz). Not to be confused with
 * @ref OgreReferenceGrid (Z-up XY viewport reference grid).
 *
 * @see OgreBillboardLine
 * @see OgreReferenceGrid
 */

#pragma once

#include <cstdint>
#include <functional>
#include <memory>
#include <string>

#include <OgreColourValue.h>
#include <OgreMaterial.h>
#include <OgreSharedPtr.h>
#include <OgreVector.h>

namespace Ogre {
class SceneManager;
class ManualObject;
class SceneNode;
class Any;
}  // namespace Ogre

namespace autoviz {
namespace rendering {

class OgreBillboardLine;

/**
 * @class OgreGrid
 * @brief Displays a grid of cells, drawn with lines.
 *
 * A grid with an identity orientation is drawn along the XZ plane.
 */
class OgreGrid {
 public:
  enum Style {
    Lines,
    Billboards,
  };

  /**
   * @param manager The scene manager this object is part of.
   * @param parent_node Parent scene node; @c nullptr uses the root node.
   * @param style Lines or Billboards.
   * @param cell_count The number of cells to draw.
   * @param cell_length The size of each cell.
   * @param line_width Line width when drawn in Billboards style.
   * @param color The color of the lines.
   */
  OgreGrid(Ogre::SceneManager* manager, Ogre::SceneNode* parent_node,
           Style style, uint32_t cell_count, float cell_length,
           float line_width, const Ogre::ColourValue& color);
  ~OgreGrid();

  OgreGrid(const OgreGrid&) = delete;
  OgreGrid& operator=(const OgreGrid&) = delete;

  /** @brief (Re)builds the grid geometry. */
  void create();

  /** @brief Returns the Ogre scene node associated with this grid. */
  Ogre::SceneNode* getSceneNode() { return scene_node_; }

  /** @brief Sets user data on all Ogre objects we own. */
  void setUserData(const Ogre::Any& data);

  void setStyle(Style style);
  Style getStyle() { return style_; }

  void setColor(const Ogre::ColourValue& color);
  Ogre::ColourValue getColor() { return color_; }

  void setCellCount(uint32_t count);
  uint32_t getCellCount() { return cell_count_; }

  void setCellLength(float len);
  float getCellLength() { return cell_length_; }

  void setLineWidth(float width);
  float getLineWidth() { return line_width_; }

  /** @brief Sets the number of extra stacked planes (height_count). */
  void setHeight(uint32_t count);
  uint32_t getHeight() { return height_count_; }

  /// Exposed for testing.
  Ogre::ManualObject* getManualObject() { return manual_object_; }
  std::shared_ptr<OgreBillboardLine> getBillboardLine() { return billboard_line_; }

 private:
  typedef std::function<void(const Ogre::Vector3& p1, const Ogre::Vector3& p2)>
      AddLineFunction;
  void createBillboardGrid() const;
  void createManualGrid() const;
  void createLines(AddLineFunction addLine) const;
  void createGridPlane(float extent, uint32_t height,
                       AddLineFunction addLine) const;
  void createVerticalLinesBetweenPlanes(float extent,
                                        AddLineFunction addLine) const;
  void addBillboardLine(const Ogre::Vector3& p1, const Ogre::Vector3& p2) const;
  void addManualLine(const Ogre::Vector3& p1, const Ogre::Vector3& p2) const;
  uint32_t numberOfVerticalLines() const;

  Ogre::SceneManager* scene_manager_;
  Ogre::SceneNode* scene_node_;        ///< Scene node this grid is attached to.
  Ogre::ManualObject* manual_object_;  ///< Manual object used to draw the grid.

  std::shared_ptr<OgreBillboardLine> billboard_line_;

  Ogre::MaterialPtr material_;
  std::string material_name_;

  Style style_;
  uint32_t cell_count_;
  float cell_length_;
  float line_width_;
  uint32_t height_count_;
  Ogre::ColourValue color_;
};

}  // namespace rendering
}  // namespace autoviz
