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

#include "autonomy/localization/atlas/map/cloud_map.hpp"

#include <algorithm>
#include <cmath>
#include <cstring>

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

}  // namespace

CloudMap::CloudMap(Options options) : options_(std::move(options)) {
    if (options_.voxel_size <= 1e-4f) {
        options_.voxel_size = 0.05f;
    }
}

void CloudMap::Clear() {
    std::lock_guard<std::mutex> lock(mutex_);
    ground_.clear();
    obstacles_.clear();
    empty_.clear();
}

std::size_t CloudMap::size() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return ground_.size() + obstacles_.size() + empty_.size();
}

std::size_t CloudMap::ground_size() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return ground_.size();
}

std::size_t CloudMap::obstacle_size() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return obstacles_.size();
}

void CloudMap::InsertLayer(VoxelHash* dst, const std::vector<Cell3D>& layer,
                           const Mat44_t& T_map_link, uint32_t default_rgb) {
    if (dst == nullptr) {
        return;
    }
    const float cell = options_.voxel_size;
    for (const auto& c : layer) {
        if (static_cast<int>(dst->size()) >= options_.max_points) {
            return;
        }
        const Eigen::Vector3f p = TransformCell(T_map_link, c);
        VoxelKey key;
        key.ix = static_cast<int>(std::floor(p.x() / cell));
        key.iy = static_cast<int>(std::floor(p.y() / cell));
        key.iz = static_cast<int>(std::floor(p.z() / cell));
        Cell3D out;
        out.x = (key.ix + 0.5f) * cell;
        out.y = (key.iy + 0.5f) * cell;
        out.z = (key.iz + 0.5f) * cell;
        out.rgb = c.rgb != 0
                      ? c.rgb
                      : (options_.colorize_layers ? default_rgb : 0);
        auto it = dst->find(key);
        if (it == dst->end()) {
            dst->emplace(key, out);
        } else if (it->second.rgb == 0 && out.rgb != 0) {
            it->second.rgb = out.rgb;
        }
    }
}

void CloudMap::Integrate(const LocalGrid& local) {
    if (local.empty_grid()) {
        return;
    }
    const Mat44_t T_map_link = OpencvPoseToRosCameraLink(local.T_wc_opencv);
    std::lock_guard<std::mutex> lock(mutex_);
    if (options_.include_ground) {
        InsertLayer(&ground_, local.ground, T_map_link, options_.ground_rgb);
    }
    if (options_.include_obstacles) {
        InsertLayer(&obstacles_, local.obstacles, T_map_link,
                    options_.obstacle_rgb);
    }
    if (options_.include_empty) {
        InsertLayer(&empty_, local.empty, T_map_link, options_.empty_rgb);
    }
}

bool CloudMap::FillMessage(const VoxelHash& voxels, const std::string& frame_id,
                           double timestamp_sec, int max_points,
                           automsgs::msgs::sensor_msgs::PointCloud2* msg) {
    if (msg == nullptr || voxels.empty()) {
        return false;
    }
    const auto sec = static_cast<int32_t>(timestamp_sec);
    auto nanosec = static_cast<uint32_t>(
        std::llround((timestamp_sec - static_cast<double>(sec)) * 1e9));
    if (nanosec >= 1000000000u) {
        msg->mutable_header()->mutable_stamp()->set_sec(sec + 1);
        msg->mutable_header()->mutable_stamp()->set_nanosec(nanosec -
                                                            1000000000u);
    } else {
        msg->mutable_header()->mutable_stamp()->set_sec(sec);
        msg->mutable_header()->mutable_stamp()->set_nanosec(nanosec);
    }
    msg->mutable_header()->set_frame_id(frame_id);

    const int count =
        std::min(static_cast<int>(voxels.size()), std::max(1, max_points));
    msg->set_height(1);
    msg->set_width(static_cast<uint32_t>(count));
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

    std::vector<float> data;
    data.resize(static_cast<std::size_t>(count) * 4);
    int i = 0;
    for (const auto& kv : voxels) {
        if (i >= count) {
            break;
        }
        data[static_cast<std::size_t>(i) * 4 + 0] = kv.second.x;
        data[static_cast<std::size_t>(i) * 4 + 1] = kv.second.y;
        data[static_cast<std::size_t>(i) * 4 + 2] = kv.second.z;
        float rgb_as_float = 0.f;
        const uint32_t rgb = kv.second.rgb;
        std::memcpy(&rgb_as_float, &rgb, sizeof(float));
        data[static_cast<std::size_t>(i) * 4 + 3] = rgb_as_float;
        ++i;
    }
    msg->set_data(reinterpret_cast<const char*>(data.data()),
                  data.size() * sizeof(float));
    return true;
}

bool CloudMap::ToMessage(const std::string& frame_id, double timestamp_sec,
                         Layer layer,
                         automsgs::msgs::sensor_msgs::PointCloud2* msg) const {
    std::lock_guard<std::mutex> lock(mutex_);
    if (layer == Layer::kGround) {
        return FillMessage(ground_, frame_id, timestamp_sec, options_.max_points,
                           msg);
    }
    if (layer == Layer::kObstacles) {
        return FillMessage(obstacles_, frame_id, timestamp_sec,
                           options_.max_points, msg);
    }
    if (layer == Layer::kEmpty) {
        return FillMessage(empty_, frame_id, timestamp_sec, options_.max_points,
                           msg);
    }
    // Merged: copy into a temporary hash (ground then obstacles overwrite).
    VoxelHash merged = ground_;
    for (const auto& kv : obstacles_) {
        merged[kv.first] = kv.second;
    }
    for (const auto& kv : empty_) {
        if (merged.find(kv.first) == merged.end()) {
            merged.emplace(kv);
        }
    }
    return FillMessage(merged, frame_id, timestamp_sec, options_.max_points,
                       msg);
}

}  // namespace map
}  // namespace atlas
}  // namespace localization
}  // namespace autonomy
