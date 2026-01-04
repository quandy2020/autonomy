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

#include "autonomy/visualization/converter/converter_detail.hpp"

namespace autonomy {
namespace visualization {
namespace converter {
namespace detail {

void SetEntityHeader(
    foxglove::schemas::SceneEntity& entity,
    const std::optional<foxglove::schemas::Timestamp>& timestamp,
    const std::optional<std::string>& frame_id) {
    if (timestamp) {
        entity.timestamp = *timestamp;
    }
    if (frame_id) {
        entity.frame_id = *frame_id;
    }
}

void SetRawImageHeader(
    foxglove::schemas::RawImage& raw_image,
    const std::optional<foxglove::schemas::Timestamp>& timestamp,
    const std::optional<std::string>& frame_id) {
    if (timestamp) {
        raw_image.timestamp = *timestamp;
    }
    if (frame_id) {
        raw_image.frame_id = *frame_id;
    }
}

void SetGridHeader(foxglove::schemas::Grid& grid,
                   const std::optional<foxglove::schemas::Timestamp>& timestamp,
                   const std::optional<std::string>& frame_id) {
    if (timestamp) {
        grid.timestamp = *timestamp;
    }
    if (frame_id) {
        grid.frame_id = *frame_id;
    }
}

void SetPointCloudHeader(
    foxglove::schemas::PointCloud& pointcloud,
    const std::optional<foxglove::schemas::Timestamp>& timestamp,
    const std::optional<std::string>& frame_id) {
    if (timestamp) {
        pointcloud.timestamp = *timestamp;
    }
    if (frame_id) {
        pointcloud.frame_id = *frame_id;
    }
}

foxglove::schemas::Pose CreatePose(
    const autonomy::commsgs::proto::geometry_msgs::Pose& pose) {
    foxglove::schemas::Pose fp;
    fp.position = foxglove::schemas::Vector3();
    fp.position->x = pose.position().x();
    fp.position->y = pose.position().y();
    fp.position->z = pose.position().z();
    fp.orientation = foxglove::schemas::Quaternion();
    fp.orientation->x = pose.orientation().x();
    fp.orientation->y = pose.orientation().y();
    fp.orientation->z = pose.orientation().z();
    fp.orientation->w = pose.orientation().w();
    return fp;
}

foxglove::schemas::Color CreateColor(double r, double g, double b, double a) {
    foxglove::schemas::Color color;
    color.r = r;
    color.g = g;
    color.b = b;
    color.a = a;
    return color;
}

}  // namespace detail
}  // namespace converter
}  // namespace visualization
}  // namespace autonomy
