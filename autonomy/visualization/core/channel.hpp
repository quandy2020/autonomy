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

#include <chrono>
#include <cstdint>
#include <memory>
#include <string>
#include <type_traits>

#include "autolink/autolink.hpp"
#include "autonomy/common/macros.hpp"
#include "autonomy/visualization/core/channel_internal.hpp"
#include "autonomy/visualization/core/channel_traits.hpp"
#include "autonomy/visualization/core/channel_writer.hpp"
#include "autonomy/visualization/core/message_traits.hpp"
#include "foxglove/schemas.hpp"
#include "google/protobuf/descriptor.h"

namespace autonomy {
namespace visualization {

/**
 * @brief Channel：Foxglove 消息通道
 *
 * 自动选择使用 Foxglove schema 或 protobuf schema
 *
 * @tparam ProtoMsgs protobuf 消息类型
 */
template <typename ProtoMsgs>
class Channel
{
public:
    AUTONOMY_SMART_PTR_DEFINITIONS(Channel)

    /// 构造函数
    /// @param topic_name topic 名称
    /// @param default_foxglove_schema 是否默认使用 Foxglove
    /// schema（已弃用，自动选择）
    explicit Channel(const std::string& topic_name,
                     bool default_foxglove_schema = false);

    /// 发布消息（自动选择 protobuf 或 Foxglove 格式）
    /// @param message 要发布的消息
    /// @return 是否发布成功
    bool Publish(const ProtoMsgs& message);

    /// 获取 topic 名称
    /// @return topic 名称
    std::string topic_name() const {
        return topic_name_;
    }

private:
    std::unique_ptr<ChannelType<ProtoMsgs>> channel_{nullptr};
    std::string topic_name_;
    ::foxglove::Schema schema_;
    std::string serialized_descriptor_storage_;
};

// Template implementation
template <typename ProtoMsgs>
Channel<ProtoMsgs>::Channel(const std::string& topic_name,
                            bool default_foxglove_schema)
    : topic_name_(topic_name) {
    using ChType = ChannelType<ProtoMsgs>;

    // Always use schema channel for types that need conversion
    if (NeedsConversion<ProtoMsgs>()) {
        // Use Foxglove schema channel (SceneUpdateChannel, GridChannel,
        // RawImageChannel)
        channel_ = detail::CreateSchemaChannel<ProtoMsgs>(topic_name_);
    } else if constexpr (std::is_same_v<ChType, ::foxglove::RawChannel>) {
        // Use RawChannel with protobuf schema only for RawChannel types
        const google::protobuf::Descriptor* descriptor =
            ProtoMsgs::descriptor();
        // Build schema storage first
        schema_ = detail::CreateProtobufSchema<ProtoMsgs>(
            descriptor, serialized_descriptor_storage_);
        channel_ = detail::CreateProtobufChannel<ProtoMsgs>(
            topic_name_, descriptor, serialized_descriptor_storage_);
    } else {
        // Fallback: try schema channel
        channel_ = detail::CreateSchemaChannel<ProtoMsgs>(topic_name_);
    }
}

template <typename ProtoMsgs>
bool Channel<ProtoMsgs>::Publish(const ProtoMsgs& message) {
    if (channel_ == nullptr) {
        AERROR << "Channel not initialized for: " << topic_name_;
        return false;
    }

    // Get current time in nanoseconds since epoch
    auto now = std::chrono::system_clock::now();
    auto duration = now.time_since_epoch();
    auto nanoseconds =
        std::chrono::duration_cast<std::chrono::nanoseconds>(duration).count();
    uint64_t timestamp_ns = static_cast<uint64_t>(nanoseconds);

    using ChType = ChannelType<ProtoMsgs>;
    using Writer = detail::ChannelPublishTraits<ChType, ProtoMsgs>;

    // 使用 trait 处理不同类型的 channel 发布
    if constexpr (NeedsConversion<ProtoMsgs>()) {
        // 对于需要转换的消息类型，使用对应的 schema channel trait
        auto* typed_channel = static_cast<ChType*>(channel_.get());
        if (typed_channel) {
            return Writer::Publish(typed_channel, message, timestamp_ns,
                                   topic_name_);
        }
        AERROR << "Channel type mismatch for converted message";
        return false;
    } else if constexpr (std::is_same_v<ChType, ::foxglove::RawChannel>) {
        // 对于 RawChannel，使用 RawChannel trait
        auto* raw_channel =
            static_cast<::foxglove::RawChannel*>(channel_.get());
        if (raw_channel) {
            return Writer::Publish(raw_channel, message, timestamp_ns,
                                   topic_name_);
        }
    }

    return false;
}

}  // namespace visualization
}  // namespace autonomy
