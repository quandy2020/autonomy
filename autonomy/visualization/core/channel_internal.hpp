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

#include <functional>
#include <memory>
#include <set>
#include <string>

#include "autolink/autolink.hpp"
#include "autonomy/visualization/core/channel_factory.hpp"
#include "autonomy/visualization/core/channel_traits.hpp"
#include "foxglove/channel.hpp"
#include "foxglove/schemas.hpp"
#include "google/protobuf/descriptor.h"
#include "google/protobuf/descriptor.pb.h"

namespace autonomy {
namespace visualization {
namespace detail {

/// 从消息描述符创建 protobuf schema
/// 递归收集所有依赖的 FileDescriptor，并序列化为 FileDescriptorSet
template <typename ProtoMsgs>
::foxglove::Schema CreateProtobufSchema(
    const google::protobuf::Descriptor* descriptor, std::string& storage) {
    ::foxglove::Schema schema;

    if (descriptor == nullptr) {
        return schema;
    }

    schema.encoding = "protobuf";
    schema.name = descriptor->full_name();

    const google::protobuf::FileDescriptor* file_descriptor =
        descriptor->file();
    if (file_descriptor == nullptr) {
        return schema;
    }

    // Recursively collect all dependencies
    std::set<const google::protobuf::FileDescriptor*> collected_files;
    std::function<void(const google::protobuf::FileDescriptor*)>
        collect_dependencies = [&](const google::protobuf::FileDescriptor* fd) {
            if (fd == nullptr ||
                collected_files.find(fd) != collected_files.end()) {
                return;
            }
            collected_files.insert(fd);
            for (int i = 0; i < fd->dependency_count(); ++i) {
                collect_dependencies(fd->dependency(i));
            }
        };

    collect_dependencies(file_descriptor);

    // Create FileDescriptorSet
    google::protobuf::FileDescriptorSet file_descriptor_set;
    for (const auto* fd : collected_files) {
        fd->CopyTo(file_descriptor_set.add_file());
    }

    // Serialize and store
    storage = file_descriptor_set.SerializeAsString();
    if (!storage.empty()) {
        schema.data = reinterpret_cast<const std::byte*>(storage.data());
        schema.data_len = storage.size();
    }

    return schema;
}

/// 创建 protobuf channel（仅用于 RawChannel 类型）
template <typename ProtoMsgs>
std::unique_ptr<ChannelType<ProtoMsgs>> CreateProtobufChannel(
    const std::string& topic_name,
    const google::protobuf::Descriptor* descriptor,
    const std::string& storage) {
    using ChType = ChannelType<ProtoMsgs>;

    // CreateProtobufChannel should only be used for RawChannel
    if constexpr (!std::is_same_v<ChType, ::foxglove::RawChannel>) {
        AERROR << "CreateProtobufChannel should only be used for RawChannel. "
               << "Use CreateSchemaChannel for schema channels.";
        return nullptr;
    }

    if (descriptor == nullptr || storage.empty()) {
        return nullptr;
    }

    ::foxglove::Schema schema;
    schema.encoding = "protobuf";
    schema.name = descriptor->full_name();
    schema.data = reinterpret_cast<const std::byte*>(storage.data());
    schema.data_len = storage.size();

    auto channel_result = ::foxglove::RawChannel::create(topic_name, "protobuf",
                                                         std::move(schema));
    if (channel_result.has_value()) {
        AINFO << "Created topic: " << topic_name << " channel successfully";
        return std::make_unique<ChType>(std::move(channel_result.value()));
    } else {
        AERROR << "Failed to create channel for: " << topic_name
               << ", error: " << static_cast<int>(channel_result.error());
        return nullptr;
    }
}

/// 创建 Foxglove schema channel（SceneUpdate/Grid/RawImage/PointCloud）
template <typename ProtoMsgs>
std::unique_ptr<ChannelType<ProtoMsgs>> CreateSchemaChannel(
    const std::string& topic_name) {
    using ChType = ChannelType<ProtoMsgs>;
    using Factory = ChannelCreateTraits<ChType>;

    // 使用 factory 创建对应的 channel
    return Factory::Create(topic_name);
}

}  // namespace detail
}  // namespace visualization
}  // namespace autonomy
