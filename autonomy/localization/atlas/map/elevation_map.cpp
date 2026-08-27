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

#include "autonomy/localization/atlas/map/elevation_map.hpp"

#include <algorithm>
#include <cmath>
#include <cstring>
#include <vector>

#include <Eigen/Core>

#include "autonomy/localization/atlas/map/local_grid_maker.hpp"

namespace autonomy {
namespace localization {
namespace atlas {
namespace map {
namespace {

Eigen::Vector3f TransformCell(const Mat44_t& T, const Cell3D& c) {
    const Eigen::Vector4d h = T * Eigen::Vector4d(c.x, c.y, c.z, 1.0);
    return Eigen::Vector3f(static_cast<float>(h.x()), static_cast<float>(h.y()),
                           static_cast<float>(h.z()));
}

void FillStamp(double timestamp_sec,
               automsgs::msgs::std_msgs::Header* header) {
    const auto sec = static_cast<int32_t>(timestamp_sec);
    auto nanosec = static_cast<uint32_t>(
        std::llround((timestamp_sec - static_cast<double>(sec)) * 1e9));
    if (nanosec >= 1000000000u) {
        header->mutable_stamp()->set_sec(sec + 1);
        header->mutable_stamp()->set_nanosec(nanosec - 1000000000u);
    } else {
        header->mutable_stamp()->set_sec(sec);
        header->mutable_stamp()->set_nanosec(nanosec);
    }
}

}  // namespace

ElevationMap::ElevationMap(Options options) : options_(std::move(options)) {
    if (options_.cell_size <= 1e-4f) {
        options_.cell_size = 0.05f;
    }
}

void ElevationMap::Clear() {
    std::lock_guard<std::mutex> lock(mutex_);
    cells_.clear();
    min_z_ = std::numeric_limits<float>::infinity();
    max_z_ = -std::numeric_limits<float>::infinity();
}

std::size_t ElevationMap::size() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return cells_.size();
}

void ElevationMap::Insert(float x, float y, float z, uint32_t rgb,
                          uint64_t node_id) {
    if (static_cast<int>(cells_.size()) >= options_.max_cells) {
        CellKey probe;
        probe.ix = static_cast<int>(std::floor(x / options_.cell_size));
        probe.iy = static_cast<int>(std::floor(y / options_.cell_size));
        if (cells_.find(probe) == cells_.end()) {
            return;
        }
    }
    CellKey key;
    key.ix = static_cast<int>(std::floor(x / options_.cell_size));
    key.iy = static_cast<int>(std::floor(y / options_.cell_size));
    auto& cell = cells_[key];
    // Prefer higher z; newer node wins on ties (RTAB-Map GridMap).
    if (z > cell.z + 1e-4f ||
        (std::fabs(z - cell.z) <= 1e-4f && node_id >= cell.node_id)) {
        cell.z = z;
        cell.node_id = node_id;
        if (rgb != 0) {
            cell.rgb = rgb;
        } else if (cell.rgb == 0) {
            cell.rgb = options_.default_rgb;
        }
    }
    min_z_ = std::min(min_z_, z);
    max_z_ = std::max(max_z_, z);
}

void ElevationMap::Integrate(const LocalGrid& local) {
    if (local.empty_grid()) {
        return;
    }
    const Mat44_t T_map_link = OpencvPoseToRosCameraLink(local.T_wc_opencv);
    std::lock_guard<std::mutex> lock(mutex_);
    auto add_layer = [&](const std::vector<Cell3D>& layer) {
        for (const auto& c : layer) {
            const Eigen::Vector3f p = TransformCell(T_map_link, c);
            Insert(p.x(), p.y(), p.z(), c.rgb, local.id);
        }
    };
    if (options_.include_ground) {
        add_layer(local.ground);
    }
    if (options_.include_obstacles) {
        add_layer(local.obstacles);
    }
}

bool ElevationMap::ToCloudMessage(
    const std::string& frame_id, double timestamp_sec,
    automsgs::msgs::sensor_msgs::PointCloud2* msg) const {
    if (msg == nullptr) {
        return false;
    }
    std::lock_guard<std::mutex> lock(mutex_);
    if (cells_.empty()) {
        return false;
    }
    const float cell = options_.cell_size;
    FillStamp(timestamp_sec, msg->mutable_header());
    msg->mutable_header()->set_frame_id(frame_id);
    msg->set_height(1);
    msg->set_width(static_cast<uint32_t>(cells_.size()));
    msg->set_is_dense(true);
    msg->set_is_bigendian(false);
    msg->set_point_step(16);
    msg->set_row_step(msg->point_step() * msg->width());
    msg->clear_fields();
    const char* names[] = {"x", "y", "z", "rgb"};
    for (int i = 0; i < 4; ++i) {
        auto* field = msg->add_fields();
        field->set_name(names[i]);
        field->set_offset(static_cast<uint32_t>(i * 4));
        field->set_datatype(automsgs::msgs::sensor_msgs::PointField::FLOAT32);
        field->set_count(1);
    }
    std::vector<float> data(cells_.size() * 4);
    std::size_t i = 0;
    for (const auto& kv : cells_) {
        data[i * 4 + 0] = (kv.first.ix + 0.5f) * cell;
        data[i * 4 + 1] = (kv.first.iy + 0.5f) * cell;
        data[i * 4 + 2] = kv.second.z;
        float rgb_as_float = 0.f;
        uint32_t rgb = kv.second.rgb != 0 ? kv.second.rgb : options_.default_rgb;
        std::memcpy(&rgb_as_float, &rgb, sizeof(float));
        data[i * 4 + 3] = rgb_as_float;
        ++i;
    }
    msg->set_data(reinterpret_cast<const char*>(data.data()),
                  data.size() * sizeof(float));
    return true;
}

bool ElevationMap::ToGridMessage(
    const std::string& frame_id, double timestamp_sec,
    automsgs::msgs::map_msgs::OccupancyGrid* msg) const {
    if (msg == nullptr) {
        return false;
    }
    std::lock_guard<std::mutex> lock(mutex_);
    if (cells_.empty() || !std::isfinite(min_z_) || !std::isfinite(max_z_)) {
        return false;
    }
    int min_ix = std::numeric_limits<int>::max();
    int min_iy = std::numeric_limits<int>::max();
    int max_ix = std::numeric_limits<int>::min();
    int max_iy = std::numeric_limits<int>::min();
    for (const auto& kv : cells_) {
        min_ix = std::min(min_ix, kv.first.ix);
        min_iy = std::min(min_iy, kv.first.iy);
        max_ix = std::max(max_ix, kv.first.ix);
        max_iy = std::max(max_iy, kv.first.iy);
    }
    min_ix -= options_.border_cells;
    min_iy -= options_.border_cells;
    max_ix += options_.border_cells;
    max_iy += options_.border_cells;
    const int width = std::min(max_ix - min_ix + 1, options_.max_width);
    const int height = std::min(max_iy - min_iy + 1, options_.max_height);
    if (width <= 0 || height <= 0) {
        return false;
    }
    const float z_range = std::max(1e-3f, max_z_ - min_z_);
    FillStamp(timestamp_sec, msg->mutable_header());
    msg->mutable_header()->set_frame_id(frame_id);
    *msg->mutable_info()->mutable_map_load_time() = msg->header().stamp();
    msg->mutable_info()->set_resolution(options_.cell_size);
    msg->mutable_info()->set_width(static_cast<uint32_t>(width));
    msg->mutable_info()->set_height(static_cast<uint32_t>(height));
    auto* origin = msg->mutable_info()->mutable_origin();
    origin->mutable_position()->set_x(min_ix * options_.cell_size);
    origin->mutable_position()->set_y(min_iy * options_.cell_size);
    origin->mutable_position()->set_z(0.0);
    origin->mutable_orientation()->set_w(1.0);
    msg->mutable_data()->Clear();
    msg->mutable_data()->Reserve(width * height);
    for (int iy = 0; iy < height; ++iy) {
        for (int ix = 0; ix < width; ++ix) {
            CellKey key{min_ix + ix, min_iy + iy};
            auto it = cells_.find(key);
            if (it == cells_.end()) {
                msg->add_data(-1);
            } else {
                const int v = static_cast<int>(std::lround(
                    (it->second.z - min_z_) / z_range * 100.f));
                msg->add_data(std::clamp(v, 0, 100));
            }
        }
    }
    return true;
}

}  // namespace map
}  // namespace atlas
}  // namespace localization
}  // namespace autonomy
