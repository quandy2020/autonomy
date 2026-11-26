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

#include <algorithm>
#include <cstdint>
#include <string>

#include <automsgs/msgs/map_msgs/occupancy_grid_cells.hpp>
#include "autolink/transport/shm/shm_conf.hpp"
#include "autonomy/localization/cartographer/node/msg_conversion.hpp"
#include "autonomy/localization/cartographer/node/submap_fetch.hpp"
#include "glog/logging.h"

namespace autonomy {
namespace localization {
namespace cartographer {
namespace node {
namespace {

constexpr int kKnownCropPadCells = 20;

size_t OccupancyTransportBudget() {
    const uint64_t cap =
        autolink::transport::ShmConf::MaxCeilingMessageSize();
    constexpr uint64_t kInfoMargin = 2ull * 1024ull * 1024ull;
    return static_cast<size_t>(cap > kInfoMargin ? cap - kInfoMargin : cap);
}

bool FitsOccupancyGridForTransport(
    const automsgs::msgs::map_msgs::OccupancyGrid& grid) {
    const size_t cells = static_cast<size_t>(std::max(0, grid.data_size()));
    // Autolink prefixes MessageInfo (~1 KiB). ByteSizeLong is the wire size.
    const size_t bytes = grid.ByteSizeLong() + 4096u;
    const size_t budget = OccupancyTransportBudget();
    if (bytes > budget) {
        LOG(ERROR) << "OccupancyGrid " << grid.info().width() << "x"
                   << grid.info().height()
                   << " res=" << grid.info().resolution() << "m cells=" << cells
                   << " bytes=" << bytes << " (cap " << budget
                   << "); skip /map publish";
        return false;
    }
    return true;
}

// PaintSubmapSlices AABB is the union of every submap texture, mostly unknown
// padding. That turns a museum floorplan into a ~6000² grid (~39 MiB) that
// Autolink SHM cannot carry. Crop to observed cells and keep a small unknown
// border so Autoviz still shows RViz-style unknown around the mapped area.
std::unique_ptr<automsgs::msgs::map_msgs::OccupancyGrid> CropOccupancyGridToKnown(
    std::unique_ptr<automsgs::msgs::map_msgs::OccupancyGrid> grid,
    int pad_cells) {
    if (!grid) {
        return nullptr;
    }
    const int width = static_cast<int>(grid->info().width());
    const int height = static_cast<int>(grid->info().height());
    const int count =
        automsgs::msgs::map_msgs::OccupancyGridCellCount(*grid);
    if (width <= 0 || height <= 0 || count < width * height) {
        return grid;
    }

    int min_x = width;
    int min_y = height;
    int max_x = -1;
    int max_y = -1;
    for (int y = 0; y < height; ++y) {
        for (int x = 0; x < width; ++x) {
            if (automsgs::msgs::map_msgs::OccupancyGridCell(
                    *grid, y * width + x) < 0) {
                continue;
            }
            min_x = std::min(min_x, x);
            min_y = std::min(min_y, y);
            max_x = std::max(max_x, x);
            max_y = std::max(max_y, y);
        }
    }
    if (max_x < min_x) {
        return nullptr;
    }

    pad_cells = std::max(0, pad_cells);
    min_x = std::max(0, min_x - pad_cells);
    min_y = std::max(0, min_y - pad_cells);
    max_x = std::min(width - 1, max_x + pad_cells);
    max_y = std::min(height - 1, max_y + pad_cells);
    const int new_width = max_x - min_x + 1;
    const int new_height = max_y - min_y + 1;
    if (new_width == width && new_height == height) {
        return grid;
    }

    auto cropped =
        std::make_unique<automsgs::msgs::map_msgs::OccupancyGrid>();
    *cropped->mutable_header() = grid->header();
    *cropped->mutable_info() = grid->info();
    cropped->mutable_info()->set_width(static_cast<uint32_t>(new_width));
    cropped->mutable_info()->set_height(static_cast<uint32_t>(new_height));
    auto* origin = cropped->mutable_info()->mutable_origin()->mutable_position();
    const double resolution = grid->info().resolution();
    origin->set_x(origin->x() + static_cast<double>(min_x) * resolution);
    origin->set_y(origin->y() + static_cast<double>(min_y) * resolution);

    automsgs::msgs::map_msgs::OccupancyGridResizeCells(
        cropped.get(), new_width * new_height, -1);
    for (int y = 0; y < new_height; ++y) {
        for (int x = 0; x < new_width; ++x) {
            const int value = automsgs::msgs::map_msgs::OccupancyGridCell(
                *grid, (min_y + y) * width + (min_x + x));
            automsgs::msgs::map_msgs::OccupancyGridSetCell(
                cropped.get(), y * new_width + x, value);
        }
    }
    LOG_EVERY_N(INFO, 10) << "Cropped occupancy " << width << "x" << height
                          << " -> " << new_width << "x" << new_height
                          << " (pad=" << pad_cells << " cells)";
    return cropped;
}

}  // namespace

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
    slice->width = fetched_texture.width;
    slice->height = fetched_texture.height;
    slice->resolution = fetched_texture.resolution;
    slice->slice_pose = fetched_texture.slice_pose;
    // Cached slices keep prior cairo_data; DrawTexture requires empty buffer.
    slice->surface = ::cartographer::io::MakeUniqueCairoSurfacePtr(nullptr);
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
    auto grid = CreateOccupancyGridMsg(painted_slices, resolution, frame_id, stamp);
    grid = CropOccupancyGridToKnown(std::move(grid), kKnownCropPadCells);
    if (grid && !FitsOccupancyGridForTransport(*grid)) {
        return nullptr;
    }
    return grid;
}

}  // namespace node
}  // namespace cartographer
}  // namespace localization
}  // namespace autonomy
