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


#include "autoviz/rendering/objects/ogre_grid.hpp"

#include <functional>
#include <memory>
#include <stdexcept>
#include <string>

#include <OgreManualObject.h>
#include <OgreMaterialManager.h>
#include <OgreSceneManager.h>
#include <OgreSceneNode.h>
#include <OgreTechnique.h>
#include <OgreVector.h>

#include "autoviz/rendering/objects/ogre_billboard_line.hpp"
#include "autoviz/rendering/ogre_material_manager.hpp"

namespace autoviz {
namespace rendering {

OgreGrid::OgreGrid(Ogre::SceneManager* scene_manager,
                   Ogre::SceneNode* parent_node, Style style,
                   uint32_t cell_count, float cell_length, float line_width,
                   const Ogre::ColourValue& color)
    : scene_manager_(scene_manager),
      scene_node_(nullptr),
      manual_object_(nullptr),
      style_(style),
      cell_count_(cell_count),
      cell_length_(cell_length),
      line_width_(line_width),
      height_count_(0),
      color_(color) {
  if (scene_manager_ == nullptr) {
    throw std::invalid_argument("SceneManager cannot be null");
  }

  static uint32_t grid_count = 0;
  const std::string grid_name = "AvizGrid" + std::to_string(grid_count++);

  manual_object_ = scene_manager_->createManualObject(grid_name);

  if (parent_node == nullptr) {
    parent_node = scene_manager_->getRootSceneNode();
  }

  scene_node_ = parent_node->createChildSceneNode();
  scene_node_->attachObject(manual_object_);

  billboard_line_ = std::make_shared<OgreBillboardLine>(scene_manager, scene_node_);

  material_name_ = grid_name + "Material";
  material_ = OgreMaterialManager::createMaterialWithNoLighting(material_name_);

  setColor(color_);
}

OgreGrid::~OgreGrid() {
  // Destroy the billboard line (and its child node) before the grid node.
  billboard_line_.reset();
  scene_manager_->destroySceneNode(scene_node_);
  scene_manager_->destroyManualObject(manual_object_);

  if (material_) {
    material_->unload();
    Ogre::MaterialManager::getSingleton().remove(
        material_name_, OgreMaterialManager::resourceGroup());
  }
}

void OgreGrid::setCellCount(uint32_t count) {
  cell_count_ = count;
  create();
}

void OgreGrid::setCellLength(float len) {
  cell_length_ = len;
  create();
}

void OgreGrid::setLineWidth(float width) {
  line_width_ = width;
  create();
}

void OgreGrid::setColor(const Ogre::ColourValue& color) {
  color_ = color;
  OgreMaterialManager::enableAlphaBlending(material_, color.a);
  create();
}

void OgreGrid::setStyle(Style style) {
  style_ = style;
  create();
}

void OgreGrid::setHeight(uint32_t height) {
  height_count_ = height;
  create();
}

void OgreGrid::create() {
  manual_object_->clear();
  billboard_line_->clear();

  if (style_ == Billboards) {
    createBillboardGrid();
  } else {
    createManualGrid();
  }
}

void OgreGrid::createManualGrid() const {
  AddLineFunction addLineFunction = std::bind(
      &OgreGrid::addManualLine, this, std::placeholders::_1,
      std::placeholders::_2);

  const uint32_t number_of_vertices_in_plane =
      cell_count_ * 4 * (height_count_ + 1);
  manual_object_->estimateVertexCount(number_of_vertices_in_plane +
                                      numberOfVerticalLines());

  manual_object_->begin(material_->getName(),
                        Ogre::RenderOperation::OT_LINE_LIST,
                        OgreMaterialManager::resourceGroup());
  createLines(addLineFunction);
  manual_object_->end();
}

void OgreGrid::createBillboardGrid() const {
  AddLineFunction addLineFunction = std::bind(
      &OgreGrid::addBillboardLine, this, std::placeholders::_1,
      std::placeholders::_2);

  billboard_line_->setColor(color_.r, color_.g, color_.b, color_.a);
  billboard_line_->setLineWidth(line_width_);
  billboard_line_->setMaxPointsPerLine(2);
  const uint32_t number_of_lines_in_plane =
      (cell_count_ + 1) * 2 * (height_count_ + 1);
  billboard_line_->setNumLines(number_of_lines_in_plane +
                               numberOfVerticalLines());

  createLines(addLineFunction);
}

uint32_t OgreGrid::numberOfVerticalLines() const {
  return (cell_count_ + 1) * (cell_count_ + 1) * height_count_;
}

void OgreGrid::createLines(AddLineFunction addLine) const {
  const float plane_extent =
      (cell_length_ * static_cast<float>(cell_count_)) / 2;

  for (uint32_t current_height = 0; current_height <= height_count_;
       ++current_height) {
    createGridPlane(plane_extent, current_height, addLine);
  }

  if (height_count_ > 0) {
    createVerticalLinesBetweenPlanes(plane_extent, addLine);
  }
}

void OgreGrid::createGridPlane(float extent, uint32_t height,
                               AddLineFunction addLine) const {
  const float real_height =
      (height_count_ / 2.0f - static_cast<float>(height)) * cell_length_;
  for (uint32_t i = 0; i <= cell_count_; i++) {
    const float inc = extent - (i * cell_length_);

    Ogre::Vector3 p1(inc, real_height, -extent);
    Ogre::Vector3 p2(inc, real_height, extent);
    Ogre::Vector3 p3(-extent, real_height, inc);
    Ogre::Vector3 p4(extent, real_height, inc);

    addLine(p1, p2);
    addLine(p3, p4);
  }
}

void OgreGrid::createVerticalLinesBetweenPlanes(float extent,
                                                AddLineFunction addLine) const {
  for (uint32_t x = 0; x <= cell_count_; ++x) {
    for (uint32_t z = 0; z <= cell_count_; ++z) {
      const float x_real = extent - x * cell_length_;
      const float z_real = extent - z * cell_length_;

      const float y_top = (height_count_ / 2.0f) * cell_length_;
      const float y_bottom = -y_top;

      Ogre::Vector3 p1(x_real, y_bottom, z_real);
      Ogre::Vector3 p2(x_real, y_top, z_real);

      addLine(p1, p2);
    }
  }
}

void OgreGrid::addManualLine(const Ogre::Vector3& p1,
                             const Ogre::Vector3& p2) const {
  manual_object_->position(p1);
  manual_object_->colour(color_);
  manual_object_->position(p2);
  manual_object_->colour(color_);
}

void OgreGrid::addBillboardLine(const Ogre::Vector3& p1,
                                const Ogre::Vector3& p2) const {
  billboard_line_->addPoint(p1);
  billboard_line_->addPoint(p2);
  billboard_line_->finishLine();
}

void OgreGrid::setUserData(const Ogre::Any& data) {
  manual_object_->getUserObjectBindings().setUserAny(data);
}

}  // namespace rendering
}  // namespace autoviz
