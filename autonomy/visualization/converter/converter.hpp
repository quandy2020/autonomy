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

#include "autonomy/visualization/converter/converter_detail.hpp"
#include "autonomy/visualization/core/channel_traits.hpp"

// Include all converter modules
#include "autonomy/visualization/converter/builtin_interfaces_converter.hpp"
#include "autonomy/visualization/converter/diagnostic_msgs_converter.hpp"
#include "autonomy/visualization/converter/geometry_msgs_converter.hpp"
#include "autonomy/visualization/converter/map_msgs_converter.hpp"
#include "autonomy/visualization/converter/pcl_msgs_converter.hpp"
#include "autonomy/visualization/converter/planning_msgs_converter.hpp"
#include "autonomy/visualization/converter/sensor_msgs_converter.hpp"
#include "autonomy/visualization/converter/shape_msgs_converter.hpp"
#include "autonomy/visualization/converter/std_msgs_converter.hpp"
#include "autonomy/visualization/converter/stereo_msgs_converter.hpp"
#include "autonomy/visualization/converter/trajectory_msgs_converter.hpp"
#include "autonomy/visualization/converter/vision_msgs_converter.hpp"
#include "autonomy/visualization/converter/visualization_msgs_converter.hpp"

namespace autonomy {
namespace visualization {

/**
 * @brief 将 commsgs 消息转换为 Foxglove 格式的统一模板接口
 *
 * 这个模板函数根据消息类型自动推断返回类型（SceneUpdate、Grid、RawImage 或
 * PointCloud）， 并调用对应的具体实现函数。实现函数分布在各个模块的转换器中。
 *
 * @tparam MsgType 消息类型（如 commsgs::proto::sensor_msgs::LaserScan）
 * @param message 要转换的消息
 * @return 对应的 Foxglove 消息类型（通过 MessageChannelTraits 推断）
 *
 * @example
 * @code
 *   commsgs::proto::sensor_msgs::LaserScan scan_msg;
 *   auto scene_update = ToFoxglove(scan_msg);  // 自动推断返回 SceneUpdate
 *
 *   commsgs::proto::map_msgs::OccupancyGrid grid_msg;
 *   auto grid = ToFoxglove(grid_msg);  // 自动推断返回 Grid
 *
 *   commsgs::proto::sensor_msgs::Image image_msg;
 *   auto raw_image = ToFoxglove(image_msg);  // 自动推断返回 RawImage
 * @endcode
 */
template <typename MsgType>
auto ToFoxglove(const MsgType& message) ->
    typename core::MessageChannelTraits<MsgType>::FoxgloveMessageType {
    // All ToFoxgloveImpl functions are in converter::impl namespace
    // They are declared in the various converter module header files
    return converter::impl::ToFoxgloveImpl(message);
}

}  // namespace visualization
}  // namespace autonomy

// Export detail namespace for backward compatibility
namespace autonomy {
namespace visualization {
namespace detail {
using converter::detail::CreateColor;
using converter::detail::CreatePose;
using converter::detail::ExtractFrameId;
using converter::detail::ExtractTimestamp;
using converter::detail::SetEntityHeader;
using converter::detail::SetGridHeader;
using converter::detail::SetPointCloudHeader;
using converter::detail::SetRawImageHeader;
}  // namespace detail
}  // namespace visualization
}  // namespace autonomy
