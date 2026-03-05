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

#include <memory>

#include "autonomy/commsgs/sensor_msgs.hpp"
#include "autonomy/tools/aviz/common/autolink_topic_display.hpp"
#include "autonomy/tools/aviz/common/properties/color_property.hpp"
#include "autonomy/tools/aviz/common/properties/float_property.hpp"
#include "autonomy/tools/aviz/common/properties/int_property.hpp"

namespace aviz {
namespace plugins {
namespace displays {

/**
 * @brief LaserScanDisplay
 * Laser scan display
 *
 * Displays sensor_msgs::LaserScan messages as point clouds
 */
class LaserScanDisplay : public AutolinkTopicDisplay<autonomy::commsgs::sensor_msgs::LaserScan> {
  Q_OBJECT

 public:
  explicit LaserScanDisplay(const QString& name = "LaserScanDisplay");
  ~LaserScanDisplay() override;

  void onInitialize() override;
  void reset() override;

 protected:
  void processMessage(const std::shared_ptr<autonomy::commsgs::sensor_msgs::LaserScan>& msg) override;

 private Q_SLOTS:
  void updateColorAndAlpha();

 private:
  void convertLaserScanToPointCloud(const autonomy::commsgs::sensor_msgs::LaserScan& scan);

  aviz::common::properties::ColorProperty* color_property_;
  aviz::common::properties::FloatProperty* alpha_property_;
  aviz::common::properties::IntProperty* buffer_size_property_;

  // Point cloud visualization (simplified - would use PointCloudCommon in full implementation)
  std::vector<Ogre::Vector3> point_cloud_points_;
  std::vector<Ogre::ColourValue> point_cloud_colors_;
};

}  // namespace displays
}  // namespace plugins
}  // namespace aviz
