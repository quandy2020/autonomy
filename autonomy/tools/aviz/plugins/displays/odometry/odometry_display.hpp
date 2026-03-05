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

#include "autonomy/commsgs/planning_msgs.hpp"
#include "autonomy/tools/aviz/common/autolink_topic_display.hpp"
#include "autonomy/tools/aviz/common/properties/color_property.hpp"
#include "autonomy/tools/aviz/common/properties/enum_property.hpp"
#include "autonomy/tools/aviz/common/properties/float_property.hpp"
#include "autonomy/tools/aviz/common/properties/int_property.hpp"

namespace aviz {
namespace rendering {
class Arrow;
class Axes;
}  // namespace rendering
}  // namespace aviz

namespace aviz {
namespace plugins {
namespace displays {

/**
 * @brief OdometryDisplay
 * Odometry display
 *
 * Accumulates and displays the pose from a planning_msgs::Odometry message
 */
class OdometryDisplay : public AutolinkTopicDisplay<autonomy::commsgs::planning_msgs::Odometry> {
  Q_OBJECT

 public:
  enum Shape {
    ArrowShape,
    AxesShape,
  };

  OdometryDisplay();
  ~OdometryDisplay() override;

  void onInitialize() override;

  void reset() override;

  // Overides of Display
  void update(float wall_dt = 0.0f, float ros_dt = 0.0f) override;

  void processMessage(const std::shared_ptr<autonomy::commsgs::planning_msgs::Odometry>& msg) override;

 protected:
  /** @brief Overridden from AutolinkTopicDisplay to get Arrow/Axes visibility correct. */
  void onEnable() override;

 public Q_SLOTS:
  void updateCovariances();

 private Q_SLOTS:
  void updateShapeChoice();

  void updateShapeVisibility();

  void updateColorAndAlpha();

  void updateArrowsGeometry();

  void updateAxisGeometry();

 private:
  void setupProperties();

  void updateArrow(const std::unique_ptr<aviz::rendering::Arrow>& arrow);

  void updateAxes(const std::unique_ptr<aviz::rendering::Axes>& axes);

  bool messageIsValid(const std::shared_ptr<autonomy::commsgs::planning_msgs::Odometry>& message);

  bool messageIsSimilarToPrevious(const std::shared_ptr<autonomy::commsgs::planning_msgs::Odometry>& message);

  std::unique_ptr<aviz::rendering::Arrow> createAndSetArrow(const Ogre::Vector3& position,
                                                            const Ogre::Quaternion& orientation, bool use_arrow);

  std::unique_ptr<aviz::rendering::Axes> createAndSetAxes(const Ogre::Vector3& position,
                                                          const Ogre::Quaternion& orientation, bool use_axes);

  void clear();

  std::deque<std::unique_ptr<aviz::rendering::Arrow>> arrows_;
  std::deque<std::unique_ptr<aviz::rendering::Axes>> axes_;

  std::shared_ptr<autonomy::commsgs::planning_msgs::Odometry> last_used_message_;

  aviz::common::properties::EnumProperty* shape_property_;

  aviz::common::properties::ColorProperty* color_property_;
  aviz::common::properties::FloatProperty* alpha_property_;
  aviz::common::properties::FloatProperty* position_tolerance_property_;
  aviz::common::properties::FloatProperty* angle_tolerance_property_;
  aviz::common::properties::IntProperty* keep_property_;

  aviz::common::properties::FloatProperty* head_radius_property_;
  aviz::common::properties::FloatProperty* head_length_property_;
  aviz::common::properties::FloatProperty* shaft_radius_property_;
  aviz::common::properties::FloatProperty* shaft_length_property_;

  aviz::common::properties::FloatProperty* axes_length_property_;
  aviz::common::properties::FloatProperty* axes_radius_property_;
};

}  // namespace displays
}  // namespace plugins
}  // namespace aviz
