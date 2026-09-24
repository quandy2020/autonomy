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

#include "autonomy/localization/atlas/map/occupancy/occupancy_map.hpp"

#include <algorithm>
#include <cmath>

namespace autonomy {
namespace localization {
namespace atlas {

OccupancyMap::OccupancyMap() : OccupancyMap(Options{}) {}

OccupancyMap::OccupancyMap(Options options) : options_(options) {
    if (options_.resolution < 0.02) {
        options_.resolution = 0.1;
    }
}

void OccupancyMap::Clear() { cells_.clear(); }

void OccupancyMap::MarkRay(int x0, int y0, int x1, int y1) {
    int x = x0;
    int y = y0;
    const int dx = std::abs(x1 - x0);
    const int dy = std::abs(y1 - y0);
    const int sx = x0 < x1 ? 1 : -1;
    const int sy = y0 < y1 ? 1 : -1;
    int err = dx - dy;
    while (x != x1 || y != y1) {
        Key key;
        key.x = x;
        key.y = y;
        auto& cell = cells_[key];
        if (cell != 100) {
            cell = 0;
        }
        const int e2 = 2 * err;
        if (e2 > -dy) {
            err -= dy;
            x += sx;
        }
        if (e2 < dx) {
            err += dx;
            y += sy;
        }
    }
    Key hit;
    hit.x = x1;
    hit.y = y1;
    cells_[hit] = 100;
}

void OccupancyMap::Integrate(const SE3& T_wb, const PointCloud& body) {
    const double resolution = options_.resolution;
    const Vec3 origin = T_wb.translation();
    const int x0 = static_cast<int>(std::floor(origin.x() / resolution));
    const int y0 = static_cast<int>(std::floor(origin.y() / resolution));
    for (const auto& point : body) {
        if (point.z < options_.min_z || point.z > options_.max_z) {
            continue;
        }
        const double range = std::hypot(point.x, point.y);
        if (range < 0.2 || range > options_.max_range) {
            continue;
        }
        const Vec3 world = T_wb * Vec3(point.x, point.y, point.z);
        const int x1 = static_cast<int>(std::floor(world.x() / resolution));
        const int y1 = static_cast<int>(std::floor(world.y() / resolution));
        MarkRay(x0, y0, x1, y1);
    }
}

void OccupancyMap::Load(const automsgs::msgs::map_msgs::OccupancyGrid& grid) {
    cells_.clear();
    if (grid.info().resolution() > 0.f) {
        options_.resolution = grid.info().resolution();
    }
    const double resolution = options_.resolution;
    if (resolution <= 0.0 || grid.info().width() == 0) {
        return;
    }
    const int origin_x = static_cast<int>(
        std::llround(grid.info().origin().position().x() / resolution));
    const int origin_y = static_cast<int>(
        std::llround(grid.info().origin().position().y() / resolution));
    const int width = static_cast<int>(grid.info().width());
    for (int index = 0; index < grid.data_size(); ++index) {
        const int value = grid.data(index);
        if (value < 0) {
            continue;
        }
        Key key;
        key.x = origin_x + (index % width);
        key.y = origin_y + (index / width);
        cells_[key] = static_cast<int8_t>(value);
    }
}

bool OccupancyMap::Fill(automsgs::msgs::map_msgs::OccupancyGrid* grid,
                        const std::string& frame) const {
    if (grid == nullptr || cells_.empty()) {
        return false;
    }
    int min_x = cells_.begin()->first.x;
    int max_x = min_x;
    int min_y = cells_.begin()->first.y;
    int max_y = min_y;
    for (const auto& cell : cells_) {
        min_x = std::min(min_x, cell.first.x);
        max_x = std::max(max_x, cell.first.x);
        min_y = std::min(min_y, cell.first.y);
        max_y = std::max(max_y, cell.first.y);
    }
    const int width = max_x - min_x + 1;
    const int height = max_y - min_y + 1;
    grid->Clear();
    grid->mutable_header()->set_frame_id(frame);
    auto* info = grid->mutable_info();
    info->set_resolution(static_cast<float>(options_.resolution));
    info->set_width(static_cast<uint32_t>(width));
    info->set_height(static_cast<uint32_t>(height));
    info->mutable_origin()->mutable_position()->set_x(min_x * options_.resolution);
    info->mutable_origin()->mutable_position()->set_y(min_y * options_.resolution);
    info->mutable_origin()->mutable_orientation()->set_w(1.0);
    grid->mutable_data()->Resize(width * height, -1);
    for (const auto& cell : cells_) {
        const int col = cell.first.x - min_x;
        const int row = cell.first.y - min_y;
        grid->set_data(row * width + col, cell.second);
    }
    return true;
}

}  // namespace atlas
}  // namespace localization
}  // namespace autonomy
