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

#include "autonomy/localization/cartographer/node/occupancy_grid_builder.hpp"

#include "autonomy/localization/cartographer/node/msg_conversion.hpp"
#include "autonomy/localization/cartographer/node/submap_fetch.hpp"

namespace autonomy {
namespace localization {
namespace cartographer {
namespace node {

bool UpdateSubmapSliceFromTextures(
    ::cartographer::io::SubmapSlice* slice,
    const ::cartographer::io::SubmapTextures& textures,
    const commsgs::geometry_msgs::Pose& submap_pose) {
    if (!slice || textures.textures.empty()) {
        return false;
    }

    const auto& fetched_texture = textures.textures.front();
    slice->pose = ToRigid3d(submap_pose);
    slice->metadata_version = textures.version;
    slice->version = textures.version;
    slice->width = fetched_texture.width;
    slice->height = fetched_texture.height;
    slice->resolution = fetched_texture.resolution;
    slice->slice_pose = fetched_texture.slice_pose;
    slice->cairo_data.clear();
    slice->surface = ::cartographer::io::DrawTexture(
        fetched_texture.pixels.intensity, fetched_texture.pixels.alpha,
        fetched_texture.width, fetched_texture.height, &slice->cairo_data);
    return slice->surface != nullptr;
}

bool UpdateSubmapSliceFromQueryResponse(
    ::cartographer::io::SubmapSlice* slice,
    const proto::SubmapQueryResponse& response,
    const commsgs::geometry_msgs::Pose& submap_pose,
    const int submap_list_version) {
    const auto textures = ParseSubmapTexturesFromResponse(response);
    if (!textures) {
        return false;
    }
    slice->metadata_version = submap_list_version;
    return UpdateSubmapSliceFromTextures(slice, *textures, submap_pose);
}

std::unique_ptr<commsgs::map_msgs::OccupancyGrid> BuildOccupancyGrid(
    const std::map<::cartographer::mapping::SubmapId,
                   ::cartographer::io::SubmapSlice>& slices,
    const double resolution, const std::string& frame_id,
    const commsgs::builtin_interfaces::Time& stamp) {
    if (slices.empty()) {
        return nullptr;
    }
    const auto painted_slices =
        ::cartographer::io::PaintSubmapSlices(slices, resolution);
    return CreateOccupancyGridMsg(painted_slices, resolution, frame_id, stamp);
}

}  // namespace node
}  // namespace cartographer
}  // namespace localization
}  // namespace autonomy
