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

#include "autonomy/tools/aviz/plugins/displays/axes/axes_display.hpp"

#include <OgreSceneManager.h>
#include <OgreSceneNode.h>

#include <memory>
#include <string>

#include "autonomy/tools/aviz/common/properties/float_property.hpp"
#include "autonomy/tools/aviz/rendering/objects/axes.hpp"

namespace aviz {
namespace plugins {
namespace displays {

using aviz::common::properties::FloatProperty;
using aviz::rendering::Axes;

AxesDisplay::AxesDisplay() : axes_(nullptr) {
  length_property_ = new FloatProperty(QString("Length"), 1.0f, QString("Length of each axis, in meters."), nullptr,
                                       SLOT(updateShape()), this);
  length_property_->setMin(0.0001f);

  radius_property_ = new FloatProperty(QString("Radius"), 0.1f, QString("Radius of each axis, in meters."), nullptr,
                                       SLOT(updateShape()), this);
  radius_property_->setMin(0.0001f);
}

AxesDisplay::~AxesDisplay() = default;

void AxesDisplay::onInitialize() {
  if (scene_manager_ && scene_node_) {
    axes_ =
        std::make_shared<Axes>(scene_manager_, scene_node_, length_property_->getFloat(), radius_property_->getFloat());
    axes_->getSceneNode()->setVisible(isEnabled());
  }
}

void AxesDisplay::onEnable() {
  aviz::common::Display::onEnable();
  if (axes_) {
    axes_->getSceneNode()->setVisible(true);
  }
}

void AxesDisplay::onDisable() {
  if (axes_) {
    axes_->getSceneNode()->setVisible(false);
  }
  aviz::common::Display::onDisable();
}

void AxesDisplay::updateShape() {
  if (axes_) {
    axes_->set(length_property_->getFloat(), radius_property_->getFloat());
    queueRender();
  }
}

void AxesDisplay::update(float wall_dt, float ros_dt) {
  Q_UNUSED(wall_dt);
  Q_UNUSED(ros_dt);
  // Axes are static at origin, no update needed
  // TODO: If frame transformation is needed, implement it here
}

}  // namespace displays
}  // namespace plugins
}  // namespace aviz
