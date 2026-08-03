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

#include "autonomy/localization/cartographer/common/port.hpp"
#include "autonomy/localization/cartographer/io/color.hpp"
#include "autonomy/localization/cartographer/io/draw_trajectories.hpp"
#include "autonomy/localization/cartographer/node/msg_conversion.hpp"
#include "autonomy/localization/cartographer/node/map_io.hpp"
#include "autonomy/localization/cartographer/node/submap_fetch.hpp"
#include "autonomy/localization/cartographer/transform/transform.hpp"

#include <filesystem>

#include <glog/logging.h>

namespace fs = std::filesystem;

namespace autonomy {
namespace localization {
namespace cartographer {
namespace node {

bool UpdateSubmapSliceFromTextures(
    ::cartographer::io::SubmapSlice* slice,
    const ::cartographer::io::SubmapTextures& textures,
    const automsgs::msgs::geometry_msgs::Pose& submap_pose) {
    if (!slice || textures.textures.empty()) {
        return false;
    }

    const auto& fetched_texture = textures.textures.front();
    slice->pose = ToRigid3d(submap_pose);
    slice->metadata_version = textures.version;
    slice->version = textures.version;
    slice->set_width(fetched_texture.width);
    slice->set_height(fetched_texture.height);
    slice->set_resolution(fetched_texture.resolution);
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
    const automsgs::msgs::geometry_msgs::Pose& submap_pose,
    const int submap_list_version) {
    const auto textures = ParseSubmapTexturesFromResponse(response);
    if (!textures) {
        return false;
    }
    slice->metadata_version = submap_list_version;
    return UpdateSubmapSliceFromTextures(slice, *textures, submap_pose);
}

std::unique_ptr<automsgs::msgs::map_msgs::OccupancyGrid> BuildOccupancyGrid(
    const std::map<::cartographer::mapping::SubmapId,
                   ::cartographer::io::SubmapSlice>& slices,
    const double resolution, const std::string& frame_id,
    const automsgs::msgs::builtin_interfaces::Time& stamp) {
    if (slices.empty()) {
        return nullptr;
    }
    const auto painted_slices =
        ::cartographer::io::PaintSubmapSlices(slices, resolution);
    return CreateOccupancyGridMsg(painted_slices, resolution, frame_id, stamp);
}

std::vector<::cartographer::mapping::proto::Trajectory> BuildTrajectoryProtos(
    const ::cartographer::mapping::MapById<::cartographer::mapping::NodeId,
                                           ::cartographer::mapping::TrajectoryNode>&
        trajectory_nodes) {
    std::map<int, ::cartographer::mapping::proto::Trajectory> trajectories_by_id;
    for (const auto& node_id_data : trajectory_nodes) {
        auto& trajectory =
            trajectories_by_id[node_id_data.id.trajectory_id];
        trajectory.set_trajectory_id(node_id_data.id.trajectory_id);
        auto* node_proto = trajectory.add_node();
        node_proto->set_node_index(node_id_data.id.node_index);
        *node_proto->mutable_pose() =
            ::cartographer::transform::ToProto(node_id_data.data.global_pose);
    }

    std::vector<::cartographer::mapping::proto::Trajectory> trajectories;
    trajectories.reserve(trajectories_by_id.size());
    for (auto& id_trajectory : trajectories_by_id) {
        if (id_trajectory.second.node_size() > 0) {
            trajectories.push_back(std::move(id_trajectory.second));
        }
    }
    return trajectories;
}

bool SaveMapImageFiles(
    const std::map<::cartographer::mapping::SubmapId,
                   ::cartographer::io::SubmapSlice>& slices,
    const double resolution, const std::string& output_directory,
    const std::string& filestem,
    const std::vector<::cartographer::mapping::proto::Trajectory>&
        trajectories) {
    if (slices.empty() || filestem.empty()) {
        return false;
    }

    auto painted_slices = ::cartographer::io::PaintSubmapSlices(slices, resolution);

    if (!trajectories.empty()) {
        const Eigen::Array2f& origin = painted_slices.origin();
        for (size_t i = 0; i < trajectories.size(); ++i) {
            ::cartographer::io::DrawTrajectory(
                trajectories[i], ::cartographer::io::GetColor(static_cast<int>(i)),
                [&origin, resolution](const ::cartographer::transform::Rigid3d& pose)
                    -> Eigen::Array2i {
                    const Eigen::Vector2d xy = pose.translation().head<2>();
                    return Eigen::Array2i(
                        ::cartographer::common::RoundToInt(xy.x() / resolution +
                                                         origin.x()),
                        ::cartographer::common::RoundToInt(-xy.y() / resolution +
                                                         origin.y()));
                },
                painted_slices.surface.get());
        }
    }

    ::cartographer::io::Image image(std::move(painted_slices.surface));

    std::error_code ec;
    fs::create_directories(output_directory, ec);
    if (ec) {
        LOG(ERROR) << "Failed to create map output directory '" << output_directory
                   << "': " << ec.message();
        return false;
    }

    const fs::path base_path = fs::path(output_directory) / filestem;
    const std::string pgm_path = (base_path.string() + ".pgm");
    const std::string yaml_path = (base_path.string() + ".yaml");
    const std::string png_path = (base_path.string() + ".png");

    ::cartographer::io::StreamFileWriter pgm_writer(pgm_path);
    WritePgm(image, resolution, &pgm_writer);

    const Eigen::Vector2d origin(
        -painted_slices.origin().x() * resolution,
        (painted_slices.origin().y() - image.height()) * resolution);

    ::cartographer::io::StreamFileWriter yaml_writer(yaml_path);
    WriteYaml(resolution, origin, fs::path(pgm_path).filename().string(),
              &yaml_writer);

    ::cartographer::io::StreamFileWriter png_writer(png_path);
    image.WritePng(&png_writer);

    LOG(INFO) << "Saved map image to " << png_path;
    return true;
}

}  // namespace node
}  // namespace cartographer
}  // namespace localization
}  // namespace autonomy
