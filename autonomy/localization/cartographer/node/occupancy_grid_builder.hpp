/*
 * Copyright 2026 The Openbot Authors
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

#include <map>
#include <memory>
#include <string>

#include "autonomy/commsgs/builtin_interfaces.hpp"
#include "autonomy/commsgs/geometry_msgs.hpp"
#include "autonomy/commsgs/map_msgs.hpp"
#include "autonomy/localization/cartographer/io/submap_painter.hpp"
#include "autonomy/localization/cartographer/mapping/id.hpp"
#include "autonomy/localization/cartographer/proto/cartographer_services.pb.h"

namespace autonomy {
namespace localization {
namespace cartographer {
namespace node {

bool UpdateSubmapSliceFromTextures(
    ::cartographer::io::SubmapSlice* slice,
    const ::cartographer::io::SubmapTextures& textures,
    const commsgs::geometry_msgs::Pose& submap_pose);

bool UpdateSubmapSliceFromQueryResponse(
    ::cartographer::io::SubmapSlice* slice,
    const proto::SubmapQueryResponse& response,
    const commsgs::geometry_msgs::Pose& submap_pose,
    int submap_list_version);

std::unique_ptr<commsgs::map_msgs::OccupancyGrid> BuildOccupancyGrid(
    const std::map<::cartographer::mapping::SubmapId,
                   ::cartographer::io::SubmapSlice>& slices,
    double resolution, const std::string& frame_id,
    const commsgs::builtin_interfaces::Time& stamp);

}  // namespace node
}  // namespace cartographer
}  // namespace localization
}  // namespace autonomy
