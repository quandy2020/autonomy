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

#include <optional>
#include <string>

#include "autonomy/commsgs/proto/geometry_msgs.pb.h"
#include "foxglove/schemas.hpp"

namespace autonomy {
namespace visualization {
namespace converter {
namespace detail {

// 从消息头提取时间戳（使用 SFINAE）
template <typename MsgType>
auto ExtractTimestamp(const MsgType& msg)
    -> decltype(msg.header().stamp(), std::optional<foxglove::schemas::Timestamp>{}) {
    if (msg.has_header() && msg.header().has_stamp()) {
        foxglove::schemas::Timestamp timestamp;
        timestamp.sec = msg.header().stamp().sec();
        timestamp.nsec = msg.header().stamp().nanosec();
        return timestamp;
    }
    return std::nullopt;
}

// 从消息头提取 frame_id（使用 SFINAE）
template <typename MsgType>
auto ExtractFrameId(const MsgType& msg) -> decltype(msg.header().frame_id(), std::optional<std::string>{}) {
    if (msg.has_header()) {
        return msg.header().frame_id();
    }
    return std::nullopt;
}

// 设置 Entity/RawImage/Grid 的头部信息
void SetEntityHeader(foxglove::schemas::SceneEntity& entity,
                     const std::optional<foxglove::schemas::Timestamp>& timestamp,
                     const std::optional<std::string>& frame_id);

void SetRawImageHeader(foxglove::schemas::RawImage& raw_image,
                       const std::optional<foxglove::schemas::Timestamp>& timestamp,
                       const std::optional<std::string>& frame_id);

void SetGridHeader(foxglove::schemas::Grid& grid, const std::optional<foxglove::schemas::Timestamp>& timestamp,
                   const std::optional<std::string>& frame_id);

void SetPointCloudHeader(foxglove::schemas::PointCloud& pointcloud,
                         const std::optional<foxglove::schemas::Timestamp>& timestamp,
                         const std::optional<std::string>& frame_id);

// 从 Pose 创建 Foxglove Pose
foxglove::schemas::Pose CreatePose(const autonomy::commsgs::proto::geometry_msgs::Pose& pose);

// 创建颜色
foxglove::schemas::Color CreateColor(double r, double g, double b, double a = 1.0);

}  // namespace detail
}  // namespace converter
}  // namespace visualization
}  // namespace autonomy
