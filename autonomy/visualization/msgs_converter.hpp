/*
 * Copyright 2025 The Openbot Authors (duyongquan)
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#pragma once

#include <opencv2/opencv.hpp>

#include <foxglove/foxglove.hpp>
#include <foxglove/context.hpp>
#include <foxglove/error.hpp>
#include <foxglove/mcap.hpp>
#include <foxglove/server.hpp>

#include "autonomy/commsgs/std_msgs.hpp"
#include "autonomy/commsgs/geometry_msgs.hpp"
#include "autonomy/commsgs/sensor_msgs.hpp"
#include "autonomy/commsgs/planning_msgs.hpp"
#include "autonomy/commsgs/map_msgs.hpp"

namespace autonomy {
namespace visualization { 

// convert msgs form commsgs [commsgs::sensor_msgs::Image] --> [::foxglove::schemas::RawImage] 
foxglove::schemas::RawImage FromCommsgs(const commsgs::sensor_msgs::Image& msgs);
foxglove::schemas::RawImage FromCommsgs(const cv::Mat& mat);

// convert msgs form commsgs [commsgs::sensor_msgs::PointCloud] --> [::foxglove::schemas::PointCloud] 
foxglove::schemas::PointCloud FromCommsgs(const commsgs::sensor_msgs::PointCloud& msgs);
foxglove::schemas::PointCloud FromCommsgs(const commsgs::sensor_msgs::PointCloud2& msgs);

// convert msgs form commsgs [commsgs::sensor_msgs::LaserScan] --> [::foxglove::schemas::LaserScan] 
foxglove::schemas::LaserScan FromCommsgs(const commsgs::sensor_msgs::LaserScan& msgs);

// convert msgs form commsgs [commsgs::geometry_msgs::Point] --> [::foxglove::schemas::Point3] 
foxglove::schemas::Point3 FromCommsgs(const commsgs::geometry_msgs::Point& msgs);

// convert msgs form commsgs [commsgs::geometry_msgs::Pose] --> [::foxglove::schemas::Pose] 
foxglove::schemas::Pose FromCommsgs(const commsgs::geometry_msgs::Pose& msgs);

// convert msgs form commsgs [commsgs::geometry_msgs::PolygonStamped] --> [::foxglove::schemas::LinePrimitive] 
foxglove::schemas::LinePrimitive FromCommsgs(const commsgs::geometry_msgs::PolygonStamped& msgs);

// convert msgs form commsgs [commsgs::map_msgs::TransformStampeds] --> [::foxglove::schemas::FrameTransforms] 
foxglove::schemas::FrameTransforms FromCommsgs(const commsgs::geometry_msgs::TransformStampeds& msgs);

// convert msgs form commsgs [commsgs::map_msgs::OccupancyGrid] --> [::foxglove::schemas::Grid] 
foxglove::schemas::Grid FromCommsgs(const commsgs::map_msgs::OccupancyGrid& msgs);

// convert msgs form commsgs [commsgs::planning_msgs::Path] --> [::foxglove::schemas::LinePrimitive] 
foxglove::schemas::LinePrimitive FromCommsgs(const commsgs::planning_msgs::Path& msgs);


}   // namespace visualization
}   // namespace autonomy
