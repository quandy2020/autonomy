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

#include <cstdint>
#include <string>

#include "autolink/autolink.hpp"
#include "autonomy/visualization/converter/converter.hpp"
#include "foxglove/channel.hpp"
#include "foxglove/schemas.hpp"

namespace autonomy {
namespace visualization {
namespace detail {

/// Channel 发布行为特征萃取
/// 用于向不同类型的 Foxglove channel 发布消息
template <typename ChType, typename ProtoMsgs>
struct ChannelPublishTraits {
    // 默认实现：不支持
    static bool Publish(ChType* channel, const ProtoMsgs& message, uint64_t timestamp_ns,
                        const std::string& topic_name) {
        (void)channel;
        (void)message;
        (void)timestamp_ns;
        (void)topic_name;
        return false;
    }
};

/// SceneUpdateChannel 特化：发布 3D 场景更新
template <typename ProtoMsgs>
struct ChannelPublishTraits<::foxglove::schemas::SceneUpdateChannel, ProtoMsgs> {
    static bool Publish(::foxglove::schemas::SceneUpdateChannel* channel, const ProtoMsgs& message,
                        uint64_t timestamp_ns, const std::string& topic_name) {
        (void)topic_name;
        if (channel) {
            auto scene_update = ToFoxglove(message);
            channel->log(scene_update, timestamp_ns);
            return true;
        }
        return false;
    }
};

/// GridChannel 特化：发布 2D 栅格地图
template <typename ProtoMsgs>
struct ChannelPublishTraits<::foxglove::schemas::GridChannel, ProtoMsgs> {
    static bool Publish(::foxglove::schemas::GridChannel* channel, const ProtoMsgs& message, uint64_t timestamp_ns,
                        const std::string& topic_name) {
        (void)topic_name;
        if (channel) {
            auto grid = ToFoxglove(message);
            channel->log(grid, timestamp_ns);
            return true;
        }
        return false;
    }
};

/// RawImageChannel 特化：发布图像数据
template <typename ProtoMsgs>
struct ChannelPublishTraits<::foxglove::schemas::RawImageChannel, ProtoMsgs> {
    static bool Publish(::foxglove::schemas::RawImageChannel* channel, const ProtoMsgs& message, uint64_t timestamp_ns,
                        const std::string& topic_name) {
        (void)topic_name;
        if (channel) {
            auto raw_image = ToFoxglove(message);
            channel->log(raw_image, timestamp_ns);
            return true;
        }
        return false;
    }
};

/// PointCloudChannel 特化：发布点云数据
template <typename ProtoMsgs>
struct ChannelPublishTraits<::foxglove::schemas::PointCloudChannel, ProtoMsgs> {
    static bool Publish(::foxglove::schemas::PointCloudChannel* channel, const ProtoMsgs& message,
                        uint64_t timestamp_ns, const std::string& topic_name) {
        (void)topic_name;
        if (channel) {
            auto pointcloud = ToFoxglove(message);
            channel->log(pointcloud, timestamp_ns);
            return true;
        }
        return false;
    }
};

/// RawChannel 特化：发布原始 protobuf 消息
template <typename ProtoMsgs>
struct ChannelPublishTraits<::foxglove::RawChannel, ProtoMsgs> {
    static bool Publish(::foxglove::RawChannel* channel, const ProtoMsgs& message, uint64_t timestamp_ns,
                        const std::string& topic_name) {
        std::string serialized = message.SerializeAsString();
        if (serialized.empty()) {
            AERROR << "Failed to serialize message for channel: " << topic_name;
            return false;
        }
        if (channel) {
            channel->log(reinterpret_cast<const std::byte*>(serialized.data()), serialized.size(), timestamp_ns);
            return true;
        }
        return false;
    }
};

}  // namespace detail
}  // namespace visualization
}  // namespace autonomy
