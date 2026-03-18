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

#include "autonomy/tools/aviz/plugins/displays/point/point_display.hpp"

#include <OgreSceneManager.h>
#include <OgreSceneNode.h>

#include <deque>
#include <memory>
#include <string>

#include "autonomy/tools/aviz/common/display_context.hpp"
#include "autonomy/tools/aviz/common/logging.hpp"
#include "autonomy/tools/aviz/common/msg_conversions.hpp"
#include "autonomy/tools/aviz/common/properties/color_property.hpp"
#include "autonomy/tools/aviz/common/properties/float_property.hpp"
#include "autonomy/tools/aviz/common/properties/int_property.hpp"
#include "autonomy/tools/aviz/common/properties/status_property.hpp"
#include "autonomy/tools/aviz/common/validate_floats.hpp"
#include "autonomy/tools/aviz/rendering/objects/shape.hpp"

namespace aviz {
namespace plugins {
namespace displays {

PointDisplay::PointDisplay(const QString& name)
    : AutolinkTopicDisplay<autonomy::commsgs::geometry_msgs::PointStamped>("aviz/Point") {
  setName(name);
  setUpProperties();
}

PointDisplay::~PointDisplay() = default;

void PointDisplay::onInitialize() { AutolinkTopicDisplay::onInitialize(); }

void PointDisplay::setUpProperties() {
  color_property_ = new aviz::common::properties::ColorProperty(
      QString("Color"), QColor(204, 41, 204), QString("Color of a point"), nullptr, SLOT(updateColorAndAlpha()), this);

  alpha_property_ = new aviz::common::properties::FloatProperty(QString("Alpha"), 1.0f,
                                                                QString("0 is fully transparent, 1.0 is fully opaque."),
                                                                nullptr, SLOT(updateColorAndAlpha()), this);

  radius_property_ = new aviz::common::properties::FloatProperty(QString("Radius"), 0.2f, QString("Radius of a point"),
                                                                 nullptr, SLOT(updateColorAndAlpha()), this);

  history_length_property_ = new aviz::common::properties::IntProperty(
      QString("History Length"), 1, QString("Number of prior measurements to display."), nullptr,
      SLOT(onlyKeepHistoryLengthNumberOfVisuals()), this);
  history_length_property_->setMin(1);
  history_length_property_->setMax(100000);
}

void PointDisplay::reset() {
  AutolinkTopicDisplay::reset();
  visuals_.clear();
}

void PointDisplay::updateColorAndAlpha() {
  float alpha = alpha_property_->getFloat();
  float radius = radius_property_->getFloat();
  Ogre::ColourValue color = color_property_->getOgreColor();

  for (auto visual : visuals_) {
    visual->setColor(color.r, color.g, color.b, alpha);
    visual->setScale(Ogre::Vector3(radius, radius, radius));
  }
  queueRender();
}

void PointDisplay::onlyKeepHistoryLengthNumberOfVisuals() {
  while (visuals_.size() > static_cast<size_t>(history_length_property_->getInt())) {
    visuals_.pop_front();
  }
  queueRender();
}

void PointDisplay::processMessage(const std::shared_ptr<autonomy::commsgs::geometry_msgs::PointStamped>& msg) {
  if (!msg || !scene_manager_ || !scene_node_) {
    return;
  }

  if (!aviz::common::validateFloat(msg->point.x) || !aviz::common::validateFloat(msg->point.y) ||
      !aviz::common::validateFloat(msg->point.z)) {
    setStatus(aviz::common::properties::StatusProperty::Error, "Topic",
              "Message contained invalid floating point values (nans or infs)");
    return;
  }

  // For now, assume identity transform (TODO: implement frame transformation)
  setTransformOk();

  if (visuals_.size() >= static_cast<size_t>(history_length_property_->getInt())) {
    visuals_.pop_front();
  }

  createNewSphereVisual(msg);
}

void PointDisplay::createNewSphereVisual(const std::shared_ptr<autonomy::commsgs::geometry_msgs::PointStamped>& msg) {
  std::shared_ptr<aviz::rendering::Shape> visual =
      std::make_shared<aviz::rendering::Shape>(aviz::rendering::Shape::Sphere, scene_manager_, scene_node_);

  float alpha = alpha_property_->getFloat();
  float radius = radius_property_->getFloat();
  Ogre::ColourValue color = color_property_->getOgreColor();

  visual->setColor(color.r, color.g, color.b, alpha);
  visual->setScale(Ogre::Vector3(radius, radius, radius));
  Ogre::Vector3 pos(static_cast<float>(msg->point.x), static_cast<float>(msg->point.y),
                    static_cast<float>(msg->point.z));
  visual->setPosition(pos);

  visuals_.push_back(visual);
  queueRender();
}

}  // namespace displays
}  // namespace plugins
}  // namespace aviz
