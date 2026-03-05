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

#pragma once

#include <map>
#include <memory>
#include <string>

#include "autonomy/commsgs/geometry_msgs.hpp"
#include "autonomy/tools/aviz/common/display.hpp"
#include "autonomy/tools/aviz/common/properties/bool_property.hpp"
#include "autonomy/tools/aviz/common/properties/color_property.hpp"
#include "autonomy/tools/aviz/common/properties/float_property.hpp"

namespace aviz {
namespace rendering {
class Axes;
class Arrow;
}  // namespace rendering
}  // namespace aviz

namespace aviz {
namespace plugins {
namespace displays {

/**
 * @brief TfDisplay
 * TF display
 *
 * Displays coordinate frame transforms (TF tree)
 */
class TfDisplay : public aviz::common::Display {
  Q_OBJECT

 public:
  explicit TfDisplay(const QString& name = "TfDisplay");
  ~TfDisplay() override;

  void onInitialize() override;
  void onEnable() override;
  void onDisable() override;
  void update(float wall_dt, float ros_dt) override;
  void reset() override;

 private Q_SLOTS:
  void updateShowNames();
  void updateShowAxes();
  void updateShowArrows();
  void updateFrameEnabled();

 private:
  void updateTransforms();
  void createFrameVisual(const std::string& frame_id, const std::string& parent_frame_id, const Ogre::Vector3& position,
                         const Ogre::Quaternion& orientation);

  aviz::common::properties::BoolProperty* show_names_property_;
  aviz::common::properties::BoolProperty* show_axes_property_;
  aviz::common::properties::BoolProperty* show_arrows_property_;
  aviz::common::properties::FloatProperty* frame_timeout_property_;
  aviz::common::properties::FloatProperty* update_interval_property_;
  aviz::common::properties::FloatProperty* axes_length_property_;
  aviz::common::properties::FloatProperty* axes_radius_property_;
  aviz::common::properties::FloatProperty* arrow_length_property_;
  aviz::common::properties::FloatProperty* arrow_radius_property_;

  // Frame visualization storage
  std::map<std::string, std::unique_ptr<aviz::rendering::Axes>> frame_axes_;
  std::map<std::string, std::unique_ptr<aviz::rendering::Arrow>> frame_arrows_;

  float last_update_time_;
};

}  // namespace displays
}  // namespace plugins
}  // namespace aviz
