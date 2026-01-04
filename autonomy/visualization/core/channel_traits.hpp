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

#include "foxglove/channel.hpp"
#include "foxglove/schemas.hpp"
#include "google/protobuf/message.h"

namespace autonomy {
namespace visualization {
namespace core {

/// Channel traits：不同 Foxglove channel 类型的特征
template <typename Channel>
struct ChannelTraits;

// RawChannel：通用 protobuf 消息
template <>
struct ChannelTraits<::foxglove::RawChannel> {
    using ChannelType = ::foxglove::RawChannel;
    using MessageType = google::protobuf::Message;
};

// SceneUpdateChannel：3D 场景可视化
template <>
struct ChannelTraits<::foxglove::schemas::SceneUpdateChannel> {
    using ChannelType = ::foxglove::schemas::SceneUpdateChannel;
    using MessageType = ::foxglove::schemas::SceneUpdate;
};

// GridChannel：2D 栅格地图
template <>
struct ChannelTraits<::foxglove::schemas::GridChannel> {
    using ChannelType = ::foxglove::schemas::GridChannel;
    using MessageType = ::foxglove::schemas::Grid;
};

// RawImageChannel：图像数据
template <>
struct ChannelTraits<::foxglove::schemas::RawImageChannel> {
    using ChannelType = ::foxglove::schemas::RawImageChannel;
    using MessageType = ::foxglove::schemas::RawImage;
};

// PointCloudChannel：点云数据
template <>
struct ChannelTraits<::foxglove::schemas::PointCloudChannel> {
    using ChannelType = ::foxglove::schemas::PointCloudChannel;
    using MessageType = ::foxglove::schemas::PointCloud;
};

/// 消息类型到 Channel 类型的特征映射（默认实现）
/// 具体特化在 message_traits.hpp 中定义
template <typename MsgType>
struct MessageChannelTraits {
    using ChannelType = ::foxglove::RawChannel;
    using FoxgloveMessageType = google::protobuf::Message;
    static constexpr bool needs_conversion = false;
};

/// 类型别名：根据消息类型获取对应的 Channel 类型
template <typename MsgType>
using ChannelType = typename MessageChannelTraits<MsgType>::ChannelType;

/// 检查消息类型是否需要转换为 Foxglove 格式
template <typename MsgType>
constexpr bool NeedsConversion() {
    return MessageChannelTraits<MsgType>::needs_conversion;
}

}  // namespace core

// 向后兼容的类型别名
using core::ChannelType;
using core::NeedsConversion;

}  // namespace visualization
}  // namespace autonomy
