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

#include <type_traits>
#include <boost/type_index.hpp>

#include "autonomy/common/logging.hpp"
#include "autonomy/commsgs/std_msgs.hpp"
#include "autonomy/commsgs/geometry_msgs.hpp"
#include "autonomy/commsgs/sensor_msgs.hpp"
#include "autonomy/commsgs/planning_msgs.hpp"
#include "autonomy/commsgs/map_msgs.hpp"

namespace autonomy {
namespace visualization { 

template<typename T>
void PrintTypeInfo(const T& param) 
{
    LOG(INFO) << "T 的类型是: " 
              << boost::typeindex::type_id_with_cvr<T>().pretty_name();
    
    LOG(INFO) << "param 的类型是: " 
              << boost::typeindex::type_id_with_cvr<decltype(param)>().pretty_name();
}

template <typename M>
struct message_traits;

template <>
struct message_traits<commsgs::geometry_msgs::Pose> {
    static constexpr const char* name = "commsgs::geometry_msgs::Pose";
    static constexpr const char* schema = "foxglove.Pose";
};

template <>
struct message_traits<commsgs::geometry_msgs::Point> {
    static constexpr const char* name = "commsgs::geometry_msgs::Point";
    static constexpr const char* schema = "foxglove.Point";
};

template <>
struct message_traits<commsgs::geometry_msgs::PolygonStamped> {
    static constexpr const char* name = "commsgs::geometry_msgs::PolygonStamped";
    static constexpr const char* schema = "foxglove.Polygon";
};

template <>
struct message_traits<commsgs::planning_msgs::Path> {
    static constexpr const char* name = "commsgs::planning_msgs::Path";
    static constexpr const char* schema = "foxglove.Path";
};

template <>
struct message_traits<commsgs::sensor_msgs::Image> {
    static constexpr const char* name = "commsgs::sensor_msgs::Image";
    static constexpr const char* schema = "foxglove.Image";
};

template <>
struct message_traits<commsgs::map_msgs::OccupancyGrid> {
    static constexpr const char* name = "commsgs::map_msgs::OccupancyGrid";
    static constexpr const char* schema = "foxglove.Map";
};

template <>
struct message_traits<commsgs::sensor_msgs::LaserScan> {
    static constexpr const char* name = "commsgs::sensor_msgs::LaserScan";
    static constexpr const char* schema = "foxglove.LaserScan";
};

template <>
struct message_traits<commsgs::planning_msgs::Odometry> {
    static constexpr const char* name = "commsgs::planning_msgs::Odometry";
    static constexpr const char* schema = "foxglove.Pose";
};

template <>
struct message_traits<commsgs::geometry_msgs::PoseStamped> {
    static constexpr const char* name = "commsgs::geometry_msgs::PoseStamped";
    static constexpr const char* schema = "foxglove.PoseStamped";
};

template <>
struct message_traits<commsgs::geometry_msgs::PoseWithCovarianceStamped> {
    static constexpr const char* name = "commsgs::geometry_msgs::PoseWithCovarianceStamped";
    static constexpr const char* schema = "foxglove.PoseWithCovarianceStamped";
};


template<typename T>
struct is_pubslish_message : std::false_type {};

template<>
struct is_pubslish_message<commsgs::sensor_msgs::Image> : std::true_type {};

template<>
struct is_pubslish_message<commsgs::sensor_msgs::LaserScan> : std::true_type {};

template<>
struct is_pubslish_message<commsgs::geometry_msgs::Pose> : std::true_type {};

template<>
struct is_pubslish_message<commsgs::geometry_msgs::Point> : std::true_type {};

template<>
struct is_pubslish_message<commsgs::geometry_msgs::PolygonStamped> : std::true_type {};

template<>
struct is_pubslish_message<commsgs::geometry_msgs::TransformStampeds> : std::true_type {};

template<>
struct is_pubslish_message<commsgs::planning_msgs::Path> : std::true_type {};

template<>
struct is_pubslish_message<commsgs::map_msgs::OccupancyGrid> : std::true_type {};

template<>
struct is_pubslish_message<commsgs::planning_msgs::Odometry> : std::true_type {};

template<>
struct is_pubslish_message<commsgs::geometry_msgs::PoseStamped> : std::true_type {};

template<>
struct is_pubslish_message<commsgs::geometry_msgs::PoseWithCovarianceStamped> : std::true_type {};

}   // namespace visualization
}   // namespace autonomy