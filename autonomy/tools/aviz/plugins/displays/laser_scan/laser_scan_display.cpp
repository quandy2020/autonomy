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

#include "autonomy/tools/aviz/plugins/displays/laser_scan/laser_scan_display.hpp"

#include <OgreSceneNode.h>

#include <cmath>
#include <vector>

#include "autonomy/tools/aviz/common/properties/color_property.hpp"
#include "autonomy/tools/aviz/common/properties/float_property.hpp"
#include "autonomy/tools/aviz/common/properties/int_property.hpp"
#include "autonomy/tools/aviz/common/properties/status_property.hpp"
#include "autonomy/tools/aviz/common/validate_floats.hpp"

namespace aviz {
namespace plugins {
namespace displays {

LaserScanDisplay::LaserScanDisplay(const QString& name)
    : AutolinkTopicDisplay<autonomy::commsgs::sensor_msgs::LaserScan>(
          QString("aviz/LaserScan")) {
    setName(name);
    color_property_ = new aviz::common::properties::ColorProperty(
        QString("Color"), QColor(255, 255, 0),
        QString("Color to draw the laser scan points."), nullptr,
        SLOT(updateColorAndAlpha()), this);

    alpha_property_ = new aviz::common::properties::FloatProperty(
        QString("Alpha"), 1.0f,
        QString("Amount of transparency to apply to the laser scan."), nullptr,
        SLOT(updateColorAndAlpha()), this);
    alpha_property_->setMin(0.0f);
    alpha_property_->setMax(1.0f);

    buffer_size_property_ = new aviz::common::properties::IntProperty(
        QString("Buffer Size"), 1, QString("Number of scans to display."),
        nullptr, SLOT(updateColorAndAlpha()), this);
    buffer_size_property_->setMin(1);
}

LaserScanDisplay::~LaserScanDisplay() = default;

void LaserScanDisplay::onInitialize() {
    AutolinkTopicDisplay::onInitialize();
}

void LaserScanDisplay::reset() {
    AutolinkTopicDisplay::reset();
    point_cloud_points_.clear();
    point_cloud_colors_.clear();
}

void LaserScanDisplay::processMessage(
    const std::shared_ptr<autonomy::commsgs::sensor_msgs::LaserScan>& msg) {
    if (!msg || !scene_manager_ || !scene_node_) {
        return;
    }

    // Validate message
    if (!aviz::common::validateFloat(msg->angle_min) ||
        !aviz::common::validateFloat(msg->angle_max) ||
        !aviz::common::validateFloat(msg->angle_increment) ||
        !aviz::common::validateFloat(msg->range_min) ||
        !aviz::common::validateFloat(msg->range_max)) {
        setStatus(
            aviz::common::properties::StatusProperty::Error, "Topic",
            "Message contained invalid floating point values (nans or infs)");
        return;
    }

    // For now, assume identity transform (TODO: implement frame transformation)
    setTransformOk();

    convertLaserScanToPointCloud(*msg);
    queueRender();
}

void LaserScanDisplay::convertLaserScanToPointCloud(
    const autonomy::commsgs::sensor_msgs::LaserScan& scan) {
    point_cloud_points_.clear();
    point_cloud_colors_.clear();

    Ogre::ColourValue color = color_property_->getOgreColor();
    color.a = alpha_property_->getFloat();

    float angle = scan.angle_min;
    for (size_t i = 0; i < scan.ranges.size(); ++i) {
        float range = scan.ranges[i];

        // Filter invalid ranges
        if (range >= scan.range_min && range <= scan.range_max &&
            std::isfinite(range)) {
            float x = range * std::cos(angle);
            float y = range * std::sin(angle);
            float z = 0.0f;  // Laser scan is typically 2D

            point_cloud_points_.push_back(Ogre::Vector3(x, y, z));
            point_cloud_colors_.push_back(color);
        }

        angle += scan.angle_increment;
    }

    // TODO: Render point cloud using Ogre (would use PointCloudCommon in full
    // implementation)
}

void LaserScanDisplay::updateColorAndAlpha() {
    queueRender();
}

}  // namespace displays
}  // namespace plugins
}  // namespace aviz
