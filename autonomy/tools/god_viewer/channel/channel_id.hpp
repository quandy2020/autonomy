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

#include "autonomy/common/macros.hpp"
#include "autonomy/commsgs/map_msgs.hpp"
#include "autonomy/tools/god_viewer/channel/channel_base.hpp"

namespace autonomy {
namespace tools { 
namespace god_viewer { 
namespace channel {

template <typename Type>
struct ChannelId
{
    using ChannelType = Type;
    std::string topic;
    ChannelType type;
    std::string description;
    double frequency;
};

using PathChannel = ChannelId<foxglove::schemas::LinePrimitiveChannel>;
using MapChannel = ChannelId<foxglove::schemas::GridChannel>;
using CostmapMapChannel = ChannelId<foxglove::schemas::GridChannel>;
using PointCloudChannel = ChannelId<foxglove::schemas::PointCloud>;
using ImageChannel = ChannelId<foxglove::schemas::RawImageChannel>;


}   // channel
}   // god_viewer
}   // namespace tools
}   // namespace autonomy