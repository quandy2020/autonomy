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

#include <memory>
#include <string>

#include "autolink/autolink.hpp"
#include "foxglove/channel.hpp"
#include "foxglove/schemas.hpp"

namespace autonomy {
namespace visualization {
namespace detail {

/// Channel 创建行为特征萃取
/// 用于创建不同类型的 Foxglove channel
template <typename ChType>
struct ChannelCreateTraits {
  // 默认实现：不支持
  static std::unique_ptr<ChType> Create(const std::string& topic_name) {
    (void)topic_name;
    return nullptr;
  }
};

/// SceneUpdateChannel 特化：用于 3D 场景可视化
template <>
struct ChannelCreateTraits<::foxglove::schemas::SceneUpdateChannel> {
  static std::unique_ptr<::foxglove::schemas::SceneUpdateChannel> Create(const std::string& topic_name) {
    auto result = ::foxglove::schemas::SceneUpdateChannel::create(topic_name);
    if (result.has_value()) {
      AINFO << "Created SceneUpdateChannel for topic: " << topic_name;
      return std::make_unique<::foxglove::schemas::SceneUpdateChannel>(std::move(result.value()));
    } else {
      AERROR << "Failed to create SceneUpdateChannel: " << static_cast<int>(result.error());
      return nullptr;
    }
  }
};

/// GridChannel 特化：用于 2D 栅格地图
template <>
struct ChannelCreateTraits<::foxglove::schemas::GridChannel> {
  static std::unique_ptr<::foxglove::schemas::GridChannel> Create(const std::string& topic_name) {
    auto result = ::foxglove::schemas::GridChannel::create(topic_name);
    if (result.has_value()) {
      AINFO << "Created GridChannel for topic: " << topic_name;
      return std::make_unique<::foxglove::schemas::GridChannel>(std::move(result.value()));
    } else {
      AERROR << "Failed to create GridChannel: " << static_cast<int>(result.error());
      return nullptr;
    }
  }
};

/// RawImageChannel 特化：用于图像数据
template <>
struct ChannelCreateTraits<::foxglove::schemas::RawImageChannel> {
  static std::unique_ptr<::foxglove::schemas::RawImageChannel> Create(const std::string& topic_name) {
    auto result = ::foxglove::schemas::RawImageChannel::create(topic_name);
    if (result.has_value()) {
      AINFO << "Created RawImageChannel for topic: " << topic_name;
      return std::make_unique<::foxglove::schemas::RawImageChannel>(std::move(result.value()));
    } else {
      AERROR << "Failed to create RawImageChannel: " << static_cast<int>(result.error());
      return nullptr;
    }
  }
};

/// PointCloudChannel 特化：用于点云数据
template <>
struct ChannelCreateTraits<::foxglove::schemas::PointCloudChannel> {
  static std::unique_ptr<::foxglove::schemas::PointCloudChannel> Create(const std::string& topic_name) {
    auto result = ::foxglove::schemas::PointCloudChannel::create(topic_name);
    if (result.has_value()) {
      AINFO << "Created PointCloudChannel for topic: " << topic_name;
      return std::make_unique<::foxglove::schemas::PointCloudChannel>(std::move(result.value()));
    } else {
      AERROR << "Failed to create PointCloudChannel: " << static_cast<int>(result.error());
      return nullptr;
    }
  }
};

/// RawChannel 特化：需要 protobuf schema，不应直接使用此 trait
/// 应使用 CreateProtobufChannel 代替
template <>
struct ChannelCreateTraits<::foxglove::RawChannel> {
  static std::unique_ptr<::foxglove::RawChannel> Create(const std::string& topic_name) {
    (void)topic_name;
    // For RawChannel, we need protobuf schema, so return nullptr
    // This should use CreateProtobufChannel instead
    return nullptr;
  }
};

}  // namespace detail
}  // namespace visualization
}  // namespace autonomy
