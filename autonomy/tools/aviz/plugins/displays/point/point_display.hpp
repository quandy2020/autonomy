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

#include <deque>
#include <memory>

#include "autonomy/commsgs/geometry_msgs.hpp"
#include "autonomy/tools/aviz/common/autolink_topic_display.hpp"
#include "autonomy/tools/aviz/common/properties/color_property.hpp"
#include "autonomy/tools/aviz/common/properties/float_property.hpp"
#include "autonomy/tools/aviz/common/properties/int_property.hpp"

namespace aviz {
namespace rendering {
class Shape;
}  // namespace rendering
}  // namespace aviz

namespace aviz {
namespace plugins {
namespace displays {

/**
 * @brief PointDisplay
 * Point (stamped) display
 *
 * Displays geometry_msgs::PointStamped messages as spheres
 */
class PointDisplay : public AutolinkTopicDisplay<autonomy::commsgs::geometry_msgs::PointStamped> {
  Q_OBJECT

 public:
  explicit PointDisplay(const QString& name = "PointDisplay");
  ~PointDisplay() override;

  void onInitialize() override;
  void reset() override;

 protected:
  void processMessage(const std::shared_ptr<autonomy::commsgs::geometry_msgs::PointStamped>& msg) override;

 private Q_SLOTS:
  void updateColorAndAlpha();
  void onlyKeepHistoryLengthNumberOfVisuals();

 private:
  void setUpProperties();
  void createNewSphereVisual(const std::shared_ptr<autonomy::commsgs::geometry_msgs::PointStamped>& msg);

  std::deque<std::shared_ptr<aviz::rendering::Shape>> visuals_;

  aviz::common::properties::ColorProperty* color_property_;
  aviz::common::properties::FloatProperty* alpha_property_;
  aviz::common::properties::FloatProperty* radius_property_;
  aviz::common::properties::IntProperty* history_length_property_;
};

}  // namespace displays
}  // namespace plugins
}  // namespace aviz
