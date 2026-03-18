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

#include "autonomy/tools/aviz/plugins/displays/path/path_display.hpp"

#include <OgreBillboardSet.h>
#include <OgreManualObject.h>
#include <OgreMaterialManager.h>
#include <OgreTechnique.h>

#include <algorithm>
#include <string>
#include <vector>

#include "autonomy/tools/aviz/common/msg_conversions.hpp"
#include "autonomy/tools/aviz/common/properties/color_property.hpp"
#include "autonomy/tools/aviz/common/properties/enum_property.hpp"
#include "autonomy/tools/aviz/common/properties/float_property.hpp"
#include "autonomy/tools/aviz/common/properties/int_property.hpp"
#include "autonomy/tools/aviz/common/properties/vector_property.hpp"
#include "autonomy/tools/aviz/rendering/material_manager.hpp"
#include "autonomy/tools/aviz/rendering/objects/arrow.hpp"
#include "autonomy/tools/aviz/rendering/objects/axes.hpp"
#include "autonomy/tools/aviz/rendering/objects/billboard_line.hpp"

namespace aviz {
namespace plugins {
namespace displays {

using aviz::common::pointMsgToOgre;
using aviz::common::quaternionMsgToOgre;
using aviz::common::properties::ColorProperty;
using aviz::common::properties::EnumProperty;
using aviz::common::properties::FloatProperty;
using aviz::common::properties::IntProperty;
using aviz::common::properties::VectorProperty;
using aviz::rendering::Arrow;
using aviz::rendering::Axes;
using aviz::rendering::BillboardLine;

PathDisplay::PathDisplay() : AutolinkTopicDisplay<autonomy::commsgs::planning_msgs::Path>("aviz/Path") {
  style_property_ = new aviz::common::properties::EnumProperty(
      QString("Line Style"), QString("Lines"), QString("The rendering operation to use to draw the grid lines."),
      nullptr, SLOT(updateStyle()), this);

  style_property_->addOption("Lines", LINES);
  style_property_->addOption("Billboards", BILLBOARDS);

  line_width_property_ = new aviz::common::properties::FloatProperty(
      QString("Line Width"), 0.03f,
      QString("The width, in meters, of each path line. Only works with the 'Billboards' style."), nullptr,
      SLOT(updateLineWidth()), this);
  line_width_property_->setMin(0.001f);
  line_width_property_->hide();

  color_property_ = new aviz::common::properties::ColorProperty(
      QString("Color"), QColor(25, 255, 0), QString("Color to draw the path."), nullptr, nullptr, this);

  alpha_property_ = new aviz::common::properties::FloatProperty(
      QString("Alpha"), 1.0f, QString("Amount of transparency to apply to the path."), nullptr, nullptr, this);

  buffer_length_property_ = new aviz::common::properties::IntProperty(
      QString("Buffer Length"), 1, QString("Number of paths to display."), nullptr, SLOT(updateBufferLength()), this);
  buffer_length_property_->setMin(1);

  offset_property_ = new aviz::common::properties::VectorProperty(
      QString("Offset"), Ogre::Vector3::ZERO,
      QString("Allows you to offset the path from the origin of the reference frame.  In meters."), nullptr,
      SLOT(updateOffset()), this);

  pose_style_property_ = new aviz::common::properties::EnumProperty(QString("Pose Style"), QString("None"),
                                                                    QString("Shape to display the pose as."), nullptr,
                                                                    SLOT(updatePoseStyle()), this);
  pose_style_property_->addOption("None", NONE);
  pose_style_property_->addOption("Axes", AXES);
  pose_style_property_->addOption("Arrows", ARROWS);

  pose_axes_length_property_ = new aviz::common::properties::FloatProperty(
      QString("Length"), 0.3f, QString("Length of the axes."), nullptr, SLOT(updatePoseAxisGeometry()), this);
  pose_axes_radius_property_ = new aviz::common::properties::FloatProperty(
      QString("Radius"), 0.03f, QString("Radius of the axes."), nullptr, SLOT(updatePoseAxisGeometry()), this);
  pose_axes_length_property_->hide();
  pose_axes_radius_property_->hide();

  pose_arrow_color_property_ = new aviz::common::properties::ColorProperty(QString("Pose Color"), QColor(255, 85, 255),
                                                                           QString("Color to draw the poses."), nullptr,
                                                                           SLOT(updatePoseArrowColor()), this);
  pose_arrow_shaft_length_property_ =
      new aviz::common::properties::FloatProperty(QString("Shaft Length"), 0.1f, QString("Length of the arrow shaft."),
                                                  nullptr, SLOT(updatePoseArrowGeometry()), this);
  pose_arrow_head_length_property_ =
      new aviz::common::properties::FloatProperty(QString("Head Length"), 0.2f, QString("Length of the arrow head."),
                                                  nullptr, SLOT(updatePoseArrowGeometry()), this);
  pose_arrow_shaft_diameter_property_ = new aviz::common::properties::FloatProperty(
      QString("Shaft Diameter"), 0.1f, QString("Diameter of the arrow shaft."), nullptr,
      SLOT(updatePoseArrowGeometry()), this);
  pose_arrow_head_diameter_property_ = new aviz::common::properties::FloatProperty(
      QString("Head Diameter"), 0.3f, QString("Diameter of the arrow head."), nullptr, SLOT(updatePoseArrowGeometry()),
      this);
  pose_arrow_color_property_->hide();
  pose_arrow_shaft_length_property_->hide();
  pose_arrow_head_length_property_->hide();
  pose_arrow_shaft_diameter_property_->hide();
  pose_arrow_head_diameter_property_->hide();

  static int count = 0;
  std::string material_name = "LinesMaterial" + std::to_string(count++);
  lines_material_ = aviz::rendering::MaterialManager::createMaterialWithNoLighting(material_name);
}

PathDisplay::~PathDisplay() {
  destroyObjects();
  destroyPoseAxesChain();
  destroyPoseArrowChain();
}

void PathDisplay::onInitialize() {
  aviz::common::Display::onInitialize();
  updateBufferLength();
}

void PathDisplay::reset() {
  aviz::common::Display::reset();
  updateBufferLength();
}

void PathDisplay::allocateAxesVector(std::vector<Axes*>& axes_vect, size_t num) {
  auto vector_size = axes_vect.size();
  if (num > vector_size) {
    axes_vect.reserve(num);
    for (auto i = vector_size; i < num; ++i) {
      axes_vect.push_back(new Axes(scene_manager_, scene_node_, pose_axes_length_property_->getFloat(),
                                   pose_axes_radius_property_->getFloat()));
    }
  } else if (num < vector_size) {
    for (auto i = num; i < vector_size; ++i) {
      delete axes_vect[i];
    }
    axes_vect.resize(num);
  }
}

void PathDisplay::allocateArrowVector(std::vector<Arrow*>& arrow_vect, size_t num) {
  auto vector_size = arrow_vect.size();
  if (num > vector_size) {
    arrow_vect.reserve(num);
    for (auto i = vector_size; i < num; ++i) {
      arrow_vect.push_back(new Arrow(scene_manager_, scene_node_));
    }
  } else if (num < vector_size) {
    for (auto i = num; i < vector_size; ++i) {
      delete arrow_vect[i];
    }
    arrow_vect.resize(num);
  }
}

void PathDisplay::destroyPoseAxesChain() {
  for (auto& axes_vect : axes_chain_) {
    allocateAxesVector(axes_vect, 0);
  }
  axes_chain_.clear();
}

void PathDisplay::destroyPoseArrowChain() {
  for (auto& arrow_vect : arrow_chain_) {
    allocateArrowVector(arrow_vect, 0);
  }
  arrow_chain_.clear();
}

void PathDisplay::updateStyle() {
  auto style = static_cast<LineStyle>(style_property_->getOptionInt());

  if (style == BILLBOARDS) {
    line_width_property_->show();
  } else {
    line_width_property_->hide();
  }

  updateBufferLength();
}

void PathDisplay::updateLineWidth() {
  auto style = static_cast<LineStyle>(style_property_->getOptionInt());
  float line_width = line_width_property_->getFloat();

  if (style == BILLBOARDS) {
    for (auto billboard_line : billboard_lines_) {
      if (billboard_line) {
        billboard_line->setLineWidth(line_width);
      }
    }
  }
  queueRender();
}

void PathDisplay::updateOffset() {
  if (scene_node_) {
    scene_node_->setPosition(offset_property_->getVector());
    queueRender();
  }
}

void PathDisplay::updatePoseStyle() {
  auto pose_style = static_cast<PoseStyle>(pose_style_property_->getOptionInt());
  switch (pose_style) {
    case AXES:
      pose_axes_length_property_->show();
      pose_axes_radius_property_->show();
      pose_arrow_color_property_->hide();
      pose_arrow_shaft_length_property_->hide();
      pose_arrow_head_length_property_->hide();
      pose_arrow_shaft_diameter_property_->hide();
      pose_arrow_head_diameter_property_->hide();
      break;
    case ARROWS:
      pose_axes_length_property_->hide();
      pose_axes_radius_property_->hide();
      pose_arrow_color_property_->show();
      pose_arrow_shaft_length_property_->show();
      pose_arrow_head_length_property_->show();
      pose_arrow_shaft_diameter_property_->show();
      pose_arrow_head_diameter_property_->show();
      break;
    default:
      pose_axes_length_property_->hide();
      pose_axes_radius_property_->hide();
      pose_arrow_color_property_->hide();
      pose_arrow_shaft_length_property_->hide();
      pose_arrow_head_length_property_->hide();
      pose_arrow_shaft_diameter_property_->hide();
      pose_arrow_head_diameter_property_->hide();
  }
  updateBufferLength();
}

void PathDisplay::updatePoseAxisGeometry() {
  for (auto& axes_vect : axes_chain_) {
    for (auto& axes : axes_vect) {
      axes->set(pose_axes_length_property_->getFloat(), pose_axes_radius_property_->getFloat());
    }
  }
  queueRender();
}

void PathDisplay::updatePoseArrowColor() {
  QColor color = pose_arrow_color_property_->getColor();

  for (auto& arrow_vect : arrow_chain_) {
    for (auto& arrow : arrow_vect) {
      arrow->setColor(color.redF(), color.greenF(), color.blueF(), 1.0f);
    }
  }
  queueRender();
}

void PathDisplay::updatePoseArrowGeometry() {
  for (auto& arrow_vect : arrow_chain_) {
    for (auto& arrow : arrow_vect) {
      arrow->set(pose_arrow_shaft_length_property_->getFloat(), pose_arrow_shaft_diameter_property_->getFloat(),
                 pose_arrow_head_length_property_->getFloat(), pose_arrow_head_diameter_property_->getFloat());
    }
  }
  queueRender();
}

void PathDisplay::destroyObjects() {
  // Destroy all simple lines, if any
  for (auto manual_object : manual_objects_) {
    if (manual_object && scene_manager_) {
      manual_object->clear();
      scene_manager_->destroyManualObject(manual_object);
    }
  }
  manual_objects_.clear();

  // Destroy all billboards, if any
  for (auto billboard_line : billboard_lines_) {
    delete billboard_line;  // also destroys the corresponding scene node
  }
  billboard_lines_.clear();
}

void PathDisplay::updateBufferLength() {
  // Destroy all path objects
  destroyObjects();

  // Destroy all axes and arrows
  destroyPoseAxesChain();
  destroyPoseArrowChain();

  if (!scene_manager_ || !scene_node_) {
    return;
  }

  // Read options
  auto buffer_length = static_cast<size_t>(buffer_length_property_->getInt());
  auto style = static_cast<LineStyle>(style_property_->getOptionInt());

  // Create new path objects
  switch (style) {
    case LINES:  // simple lines with fixed width of 1px
      manual_objects_.reserve(buffer_length);
      for (size_t i = 0; i < buffer_length; i++) {
        auto manual_object = scene_manager_->createManualObject();
        manual_object->setDynamic(true);
        scene_node_->attachObject(manual_object);

        manual_objects_.push_back(manual_object);
      }
      break;

    case BILLBOARDS:  // billboards with configurable width
      billboard_lines_.reserve(buffer_length);
      for (size_t i = 0; i < buffer_length; i++) {
        auto billboard_line = new BillboardLine(scene_manager_, scene_node_);
        billboard_lines_.push_back(billboard_line);
      }
      break;
  }
  axes_chain_.resize(buffer_length);
  arrow_chain_.resize(buffer_length);
}

void PathDisplay::processMessage(const std::shared_ptr<autonomy::commsgs::planning_msgs::Path>& msg) {
  if (!msg || !scene_manager_ || !scene_node_) {
    return;
  }

  // Calculate index of oldest element in cyclic buffer
  static size_t messages_received_ = 0;
  size_t bufferIndex = messages_received_ % buffer_length_property_->getInt();
  messages_received_++;

  auto style = static_cast<LineStyle>(style_property_->getOptionInt());
  Ogre::ManualObject* manual_object = nullptr;
  BillboardLine* billboard_line = nullptr;

  // Delete oldest element
  switch (style) {
    case LINES:
      if (bufferIndex < manual_objects_.size()) {
        manual_object = manual_objects_[bufferIndex];
        manual_object->clear();
      }
      break;

    case BILLBOARDS:
      if (bufferIndex < billboard_lines_.size()) {
        billboard_line = billboard_lines_[bufferIndex];
        billboard_line->clear();
      }
      break;
  }

  if (msg->poses.empty()) {
    return;
  }

  // For now, assume identity transform (TODO: implement frame transformation)
  Ogre::Vector3 position(0, 0, 0);
  Ogre::Quaternion orientation(1, 0, 0, 0);
  Ogre::Matrix4 transform(orientation);
  transform.setTrans(position);

  switch (style) {
    case LINES:
      if (manual_object) {
        updateManualObject(manual_object, *msg, transform);
      }
      break;

    case BILLBOARDS:
      if (billboard_line) {
        updateBillBoardLine(billboard_line, *msg, transform);
      }
      break;
  }
  updatePoseMarkers(bufferIndex, *msg, transform);

  queueRender();
}

void PathDisplay::updateManualObject(Ogre::ManualObject* manual_object,
                                     const autonomy::commsgs::planning_msgs::Path& msg,
                                     const Ogre::Matrix4& transform) {
  auto color = color_property_->getOgreColor();
  color.a = alpha_property_->getFloat();

  manual_object->estimateVertexCount(msg.poses.size());
  manual_object->begin(lines_material_->getName(), Ogre::RenderOperation::OT_LINE_STRIP, "aviz_rendering");

  for (const auto& pose_stamped : msg.poses) {
    Ogre::Vector3 pos = transform * pointMsgToOgre(pose_stamped.pose.position);
    manual_object->position(pos);
    aviz::rendering::MaterialManager::enableAlphaBlending(lines_material_, color.a);
    manual_object->colour(color);
  }

  manual_object->end();
}

void PathDisplay::updateBillBoardLine(BillboardLine* billboard_line, const autonomy::commsgs::planning_msgs::Path& msg,
                                      const Ogre::Matrix4& transform) {
  auto color = color_property_->getOgreColor();
  color.a = alpha_property_->getFloat();

  billboard_line->setNumLines(1);
  billboard_line->setMaxPointsPerLine(static_cast<uint32_t>(msg.poses.size()));
  billboard_line->setLineWidth(line_width_property_->getFloat());

  for (const auto& pose_stamped : msg.poses) {
    Ogre::Vector3 xpos = transform * pointMsgToOgre(pose_stamped.pose.position);
    billboard_line->addPoint(xpos, color);
  }
}

void PathDisplay::updatePoseMarkers(size_t buffer_index, const autonomy::commsgs::planning_msgs::Path& msg,
                                    const Ogre::Matrix4& transform) {
  auto pose_style = static_cast<PoseStyle>(pose_style_property_->getOptionInt());
  if (buffer_index >= axes_chain_.size() || buffer_index >= arrow_chain_.size()) {
    return;
  }
  auto& arrow_vect = arrow_chain_[buffer_index];
  auto& axes_vect = axes_chain_[buffer_index];

  if (pose_style == AXES) {
    updateAxesMarkers(axes_vect, msg, transform);
  }
  if (pose_style == ARROWS) {
    updateArrowMarkers(arrow_vect, msg, transform);
  }
}

void PathDisplay::updateAxesMarkers(std::vector<Axes*>& axes_vect, const autonomy::commsgs::planning_msgs::Path& msg,
                                    const Ogre::Matrix4& transform) {
  auto num_points = msg.poses.size();
  allocateAxesVector(axes_vect, num_points);
  for (size_t i = 0; i < num_points; ++i) {
    const auto& pos = msg.poses[i].pose.position;
    axes_vect[i]->setPosition(transform * pointMsgToOgre(pos));
    Ogre::Quaternion orientation(quaternionMsgToOgre(msg.poses[i].pose.orientation));

    // Extract the rotation part of the transformation matrix as a quaternion
    Ogre::Matrix3 rot3x3;
    rot3x3[0][0] = transform[0][0];
    rot3x3[0][1] = transform[0][1];
    rot3x3[0][2] = transform[0][2];
    rot3x3[1][0] = transform[1][0];
    rot3x3[1][1] = transform[1][1];
    rot3x3[1][2] = transform[1][2];
    rot3x3[2][0] = transform[2][0];
    rot3x3[2][1] = transform[2][1];
    rot3x3[2][2] = transform[2][2];
    Ogre::Quaternion transform_orientation;
    transform_orientation.FromRotationMatrix(rot3x3);

    axes_vect[i]->setOrientation(transform_orientation * orientation);
  }
}

void PathDisplay::updateArrowMarkers(std::vector<Arrow*>& arrow_vect, const autonomy::commsgs::planning_msgs::Path& msg,
                                     const Ogre::Matrix4& transform) {
  auto num_points = msg.poses.size();
  allocateArrowVector(arrow_vect, num_points);
  for (size_t i = 0; i < num_points; ++i) {
    QColor color = pose_arrow_color_property_->getColor();
    arrow_vect[i]->setColor(color.redF(), color.greenF(), color.blueF(), 1.0f);

    arrow_vect[i]->set(pose_arrow_shaft_length_property_->getFloat(), pose_arrow_shaft_diameter_property_->getFloat(),
                       pose_arrow_head_length_property_->getFloat(), pose_arrow_head_diameter_property_->getFloat());
    const auto& pos = msg.poses[i].pose.position;
    arrow_vect[i]->setPosition(transform * pointMsgToOgre(pos));
    Ogre::Quaternion orientation(quaternionMsgToOgre(msg.poses[i].pose.orientation));

    // Extract the rotation part of the transformation matrix as a quaternion
    Ogre::Matrix3 rot3x3;
    rot3x3[0][0] = transform[0][0];
    rot3x3[0][1] = transform[0][1];
    rot3x3[0][2] = transform[0][2];
    rot3x3[1][0] = transform[1][0];
    rot3x3[1][1] = transform[1][1];
    rot3x3[1][2] = transform[1][2];
    rot3x3[2][0] = transform[2][0];
    rot3x3[2][1] = transform[2][1];
    rot3x3[2][2] = transform[2][2];
    Ogre::Quaternion transform_orientation;
    transform_orientation.FromRotationMatrix(rot3x3);

    Ogre::Vector3 dir(1, 0, 0);
    dir = transform_orientation * orientation * dir;
    arrow_vect[i]->setDirection(dir);
  }
}

}  // namespace displays
}  // namespace plugins
}  // namespace aviz
