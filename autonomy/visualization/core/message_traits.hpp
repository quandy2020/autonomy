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

#include "autonomy/commsgs/proto/geometry_msgs.pb.h"
#include "autonomy/commsgs/proto/map_msgs.pb.h"
#include "autonomy/commsgs/proto/planning_msgs.pb.h"
#include "autonomy/commsgs/proto/sensor_msgs.pb.h"
#include "autonomy/commsgs/proto/visualization_msgs.pb.h"
#include "autonomy/visualization/core/channel_traits.hpp"
#include "foxglove/schemas.hpp"

namespace autonomy {
namespace visualization {
namespace core {

// MessageChannelTraits 已在 channel_traits.hpp 中声明，这里只需要特化

/// 辅助宏：定义 SceneUpdateChannel 类型的 traits 特化
#define SCENE_UPDATE_TRAIT(MSG)                                       \
    template <>                                                       \
    struct MessageChannelTraits<MSG> {                                \
        using ChannelType = ::foxglove::schemas::SceneUpdateChannel;  \
        using FoxgloveMessageType = ::foxglove::schemas::SceneUpdate; \
        static constexpr bool needs_conversion = true;                \
    }

// Planning messages
SCENE_UPDATE_TRAIT(commsgs::proto::planning_msgs::Path);
SCENE_UPDATE_TRAIT(commsgs::proto::planning_msgs::Odometry);

// Sensor messages
SCENE_UPDATE_TRAIT(commsgs::proto::sensor_msgs::LaserScan);
SCENE_UPDATE_TRAIT(commsgs::proto::sensor_msgs::Imu);
SCENE_UPDATE_TRAIT(commsgs::proto::sensor_msgs::Range);

// Geometry messages
SCENE_UPDATE_TRAIT(commsgs::proto::geometry_msgs::PoseStamped);
SCENE_UPDATE_TRAIT(commsgs::proto::geometry_msgs::PoseArray);
SCENE_UPDATE_TRAIT(commsgs::proto::geometry_msgs::TransformStamped);
SCENE_UPDATE_TRAIT(commsgs::proto::geometry_msgs::TransformStampeds);
SCENE_UPDATE_TRAIT(commsgs::proto::geometry_msgs::Twist);
SCENE_UPDATE_TRAIT(commsgs::proto::geometry_msgs::TwistStamped);

// Visualization messages
SCENE_UPDATE_TRAIT(commsgs::proto::visualization_msgs::Marker);
SCENE_UPDATE_TRAIT(commsgs::proto::visualization_msgs::MarkerArray);

#undef SCENE_UPDATE_TRAIT

/// GridChannel traits 特化
template <>
struct MessageChannelTraits<commsgs::proto::map_msgs::OccupancyGrid> {
    using ChannelType = ::foxglove::schemas::GridChannel;
    using FoxgloveMessageType = ::foxglove::schemas::Grid;
    static constexpr bool needs_conversion = true;
};

/// 辅助宏：定义 RawImageChannel 类型的 traits 特化
#define RAW_IMAGE_TRAIT(MSG)                                       \
    template <>                                                    \
    struct MessageChannelTraits<MSG> {                             \
        using ChannelType = ::foxglove::schemas::RawImageChannel;  \
        using FoxgloveMessageType = ::foxglove::schemas::RawImage; \
        static constexpr bool needs_conversion = true;             \
    }

// Sensor image messages
RAW_IMAGE_TRAIT(commsgs::proto::sensor_msgs::Image);
RAW_IMAGE_TRAIT(commsgs::proto::sensor_msgs::CompressedImage);

#undef RAW_IMAGE_TRAIT

/// 辅助宏：定义 PointCloudChannel 类型的 traits 特化
#define POINT_CLOUD_TRAIT(MSG)                                       \
    template <>                                                      \
    struct MessageChannelTraits<MSG> {                               \
        using ChannelType = ::foxglove::schemas::PointCloudChannel;  \
        using FoxgloveMessageType = ::foxglove::schemas::PointCloud; \
        static constexpr bool needs_conversion = true;               \
    }

// Sensor point cloud messages
POINT_CLOUD_TRAIT(commsgs::proto::sensor_msgs::PointCloud2);
POINT_CLOUD_TRAIT(commsgs::proto::sensor_msgs::PointCloud);

#undef POINT_CLOUD_TRAIT

}  // namespace core
}  // namespace visualization
}  // namespace autonomy
