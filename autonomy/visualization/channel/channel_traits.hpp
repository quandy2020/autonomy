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

#include "autonomy/commsgs/std_msgs.hpp"
#include "autonomy/commsgs/sensor_msgs.hpp"
#include "autonomy/commsgs/builtin_interfaces.hpp"
#include "autonomy/commsgs/geometry_msgs.hpp"
#include "autonomy/commsgs/planning_msgs.hpp"
#include "autonomy/commsgs/map_msgs.hpp"

#include "autonomy/visualization/displays/arrow.hpp"
#include "autonomy/visualization/displays/axes.hpp"
#include "autonomy/visualization/displays/cube.hpp"
#include "autonomy/visualization/displays/cylinder.hpp"
#include "autonomy/visualization/displays/image.hpp"
#include "autonomy/visualization/displays/map.hpp"
#include "autonomy/visualization/displays/laser_scan.hpp"
#include "autonomy/visualization/displays/path.hpp"
#include "autonomy/visualization/displays/point.hpp"
#include "autonomy/visualization/displays/pointcloud.hpp"
#include "autonomy/visualization/displays/polygon.hpp"
#include "autonomy/visualization/displays/pose.hpp"
#include "autonomy/visualization/displays/robot_model.hpp"
#include "autonomy/visualization/displays/sphere.hpp"
#include "autonomy/visualization/displays/tf.hpp"

namespace autonomy {
namespace visualization { 
namespace channel { 

template <typename T>
struct ChannelTraits;

// template<> struct ChannelTraits<commsgs::sensor_msgs::Image> {
//     using type = ArrowHandler;
// };

template<> struct ChannelTraits<commsgs::sensor_msgs::Image> {
    using type = displays::ImageHandler;
};

template<> struct ChannelTraits<cv::Mat> {
    using type = displays::ImageHandler;
};


template<> struct ChannelTraits<commsgs::sensor_msgs::LaserScan> {
    using type = displays::LaserScanHandler;
};

template<> struct ChannelTraits<commsgs::sensor_msgs::PointCloud> {
    using type = displays::PointHandler;
};

template<> struct ChannelTraits<commsgs::sensor_msgs::PointCloud2> {
    using type = displays::PointHandler;
};

template<> struct ChannelTraits<commsgs::map_msgs::OccupancyGrid> {
    using type = displays::MapHandler;
};

template<> struct ChannelTraits<commsgs::planning_msgs::Path> {
    using type = displays::PathHandler;
};

template<> struct ChannelTraits<commsgs::geometry_msgs::Point> {
    using type = displays::PointHandler;
};

template<> struct ChannelTraits<commsgs::geometry_msgs::PolygonStamped> {
    using type = displays::PolygonHandler;
};

template<> struct ChannelTraits<commsgs::geometry_msgs::Pose> {
    using type = displays::PoseHandler;
};

template<> struct ChannelTraits<commsgs::geometry_msgs::PoseStamped> {
    using type = displays::PoseHandler;
};

template<> struct ChannelTraits<commsgs::geometry_msgs::PoseWithCovarianceStamped> {
    using type = displays::PoseHandler;
};

template<> struct ChannelTraits<commsgs::planning_msgs::Odometry> {
    using type = displays::PoseHandler;
};

template<> struct ChannelTraits<commsgs::geometry_msgs::TransformStampeds> {
    using type = displays::FrameTransformsHandler;
};


}   // namespace channel
}   // namespace visualization
}   // namespace autonomy