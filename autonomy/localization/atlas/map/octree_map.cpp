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

#include "autonomy/localization/atlas/map/octree_map.hpp"

#include <algorithm>
#include <cmath>
#include <cstring>
#include <limits>
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

OctreeMap::OctreeMap(Options options) : options_(std::move(options)) {
    if (options_.voxel_size <= 1e-4f) {
        options_.voxel_size = 0.05f;
    }
}

void OctreeMap::Clear() {
    std::lock_guard<std::mutex> lock(mutex_);
    voxels_.clear();
}

float OctreeMap::LogOdds(float p) const {
    p = std::clamp(p, 1e-4f, 1.f - 1e-4f);
    return std::log(p / (1.f - p));
}

float OctreeMap::ClampLogOdds(float l) const {
    return std::clamp(l, LogOdds(options_.clamping_min),
                      LogOdds(options_.clamping_max));
}

float OctreeMap::Prob(float log_odds) const {
    return 1.f - 1.f / (1.f + std::exp(log_odds));
}

void OctreeMap::UpdateVoxel(const Eigen::Vector3f& p, float delta, CellType type,
                            uint32_t rgb) {
    if (static_cast<int>(voxels_.size()) >= options_.max_voxels &&
        voxels_.find(VoxelKey{
            static_cast<int>(std::floor(p.x() / options_.voxel_size)),
            static_cast<int>(std::floor(p.y() / options_.voxel_size)),
            static_cast<int>(std::floor(p.z() / options_.voxel_size))}) ==
            voxels_.end()) {
        return;
    }
    VoxelKey key;
    key.ix = static_cast<int>(std::floor(p.x() / options_.voxel_size));
    key.iy = static_cast<int>(std::floor(p.y() / options_.voxel_size));
    key.iz = static_cast<int>(std::floor(p.z() / options_.voxel_size));
    auto& v = voxels_[key];
    v.log_odds = ClampLogOdds(v.log_odds + delta);
    // Prefer stronger labels: obstacle > ground > empty.
    if (type == CellType::kObstacle ||
        (type == CellType::kGround && v.type != CellType::kObstacle) ||
        (type == CellType::kEmpty && v.type == CellType::kUnknown)) {
        v.type = type;
    }
    if (rgb != 0 && v.rgb == 0) {
        v.rgb = rgb;
    }
}

void OctreeMap::Integrate(const LocalGrid& local) {
    if (local.empty_grid()) {
        return;
    }
    const Mat44_t T_map_link = OpencvPoseToRosCameraLink(local.T_wc_opencv);
    const float hit = LogOdds(options_.prob_hit);
    const float miss = LogOdds(options_.prob_miss);
    std::lock_guard<std::mutex> lock(mutex_);
    for (const auto& c : local.empty) {
        UpdateVoxel(TransformCell(T_map_link, c), miss, CellType::kEmpty, 0);
    }
    for (const auto& c : local.ground) {
        UpdateVoxel(TransformCell(T_map_link, c), hit, CellType::kGround,
                    c.rgb);
    }
    for (const auto& c : local.obstacles) {
        UpdateVoxel(TransformCell(T_map_link, c), hit, CellType::kObstacle,
                    c.rgb);
    }
}

std::size_t OctreeMap::occupied_size() const {
    std::lock_guard<std::mutex> lock(mutex_);
    std::size_t n = 0;
    for (const auto& kv : voxels_) {
        if (Prob(kv.second.log_odds) >= options_.occupancy_thr &&
            (kv.second.type == CellType::kObstacle ||
             kv.second.type == CellType::kGround)) {
            ++n;
        }
    }
    return n;
}

std::size_t OctreeMap::ground_size() const {
    std::lock_guard<std::mutex> lock(mutex_);
    std::size_t n = 0;
    for (const auto& kv : voxels_) {
        if (kv.second.type == CellType::kGround &&
            Prob(kv.second.log_odds) >= options_.occupancy_thr) {
            ++n;
        }
    }
    return n;
}

std::size_t OctreeMap::empty_size() const {
    std::lock_guard<std::mutex> lock(mutex_);
    std::size_t n = 0;
    for (const auto& kv : voxels_) {
        if (kv.second.type == CellType::kEmpty ||
            Prob(kv.second.log_odds) < options_.occupancy_thr) {
            ++n;
        }
    }
    return n;
}

bool OctreeMap::ToCloudMessage(
    const std::string& frame_id, double timestamp_sec, Layer layer,
    automsgs::msgs::sensor_msgs::PointCloud2* msg) const {
    if (msg == nullptr) {
        return false;
    }
    std::lock_guard<std::mutex> lock(mutex_);
    std::vector<Cell3D> pts;
    pts.reserve(voxels_.size());
    const float cell = options_.voxel_size;
    for (const auto& kv : voxels_) {
        const bool occ = Prob(kv.second.log_odds) >= options_.occupancy_thr;
        bool keep = false;
        uint32_t rgb = kv.second.rgb;
        if (layer == Layer::kGround) {
            keep = occ && kv.second.type == CellType::kGround;
            if (rgb == 0 && options_.colorize) {
                rgb = options_.ground_rgb;
            }
        } else if (layer == Layer::kEmpty) {
            keep = kv.second.type == CellType::kEmpty || !occ;
            if (rgb == 0 && options_.colorize) {
                rgb = options_.empty_rgb;
            }
        } else {
            keep = occ && (kv.second.type == CellType::kObstacle ||
                           kv.second.type == CellType::kGround);
            if (rgb == 0 && options_.colorize) {
                rgb = kv.second.type == CellType::kGround
                          ? options_.ground_rgb
                          : options_.obstacle_rgb;
            }
        }
        if (!keep) {
            continue;
        }
        Cell3D c;
        c.x = (kv.first.ix + 0.5f) * cell;
        c.y = (kv.first.iy + 0.5f) * cell;
        c.z = (kv.first.iz + 0.5f) * cell;
        c.rgb = rgb;
        pts.push_back(c);
    }
    if (pts.empty()) {
        return false;
    }
    FillStamp(timestamp_sec, msg->mutable_header());
    msg->mutable_header()->set_frame_id(frame_id);
    msg->set_height(1);
    msg->set_width(static_cast<uint32_t>(pts.size()));
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
    std::vector<float> data(pts.size() * 4);
    for (std::size_t i = 0; i < pts.size(); ++i) {
        data[i * 4 + 0] = pts[i].x;
        data[i * 4 + 1] = pts[i].y;
        data[i * 4 + 2] = pts[i].z;
        float rgb_as_float = 0.f;
        std::memcpy(&rgb_as_float, &pts[i].rgb, sizeof(float));
        data[i * 4 + 3] = rgb_as_float;
    }
    msg->set_data(reinterpret_cast<const char*>(data.data()),
                  data.size() * sizeof(float));
    return true;
}

bool OctreeMap::ToProjectionMessage(
    const std::string& frame_id, double timestamp_sec,
    automsgs::msgs::map_msgs::OccupancyGrid* msg) const {
    if (msg == nullptr) {
        return false;
    }
    std::lock_guard<std::mutex> lock(mutex_);
    if (voxels_.empty()) {
        return false;
    }
    int min_ix = std::numeric_limits<int>::max();
    int min_iy = std::numeric_limits<int>::max();
    int max_ix = std::numeric_limits<int>::min();
    int max_iy = std::numeric_limits<int>::min();
    for (const auto& kv : voxels_) {
        min_ix = std::min(min_ix, kv.first.ix);
        min_iy = std::min(min_iy, kv.first.iy);
        max_ix = std::max(max_ix, kv.first.ix);
        max_iy = std::max(max_iy, kv.first.iy);
    }
    const int width = max_ix - min_ix + 1;
    const int height = max_iy - min_iy + 1;
    if (width <= 0 || height <= 0 ||
        width > 2000 || height > 2000) {
        return false;
    }
    std::vector<int8_t> data(static_cast<std::size_t>(width * height), -1);
    for (const auto& kv : voxels_) {
        const int ix = kv.first.ix - min_ix;
        const int iy = kv.first.iy - min_iy;
        const int idx = iy * width + ix;
        const bool occ = Prob(kv.second.log_odds) >= options_.occupancy_thr &&
                         (kv.second.type == CellType::kObstacle ||
                          kv.second.type == CellType::kGround);
        if (occ) {
            data[static_cast<std::size_t>(idx)] = 100;
        } else if (data[static_cast<std::size_t>(idx)] < 0) {
            data[static_cast<std::size_t>(idx)] = 0;
        }
    }
    FillStamp(timestamp_sec, msg->mutable_header());
    msg->mutable_header()->set_frame_id(frame_id);
    *msg->mutable_info()->mutable_map_load_time() = msg->header().stamp();
    msg->mutable_info()->set_resolution(options_.voxel_size);
    msg->mutable_info()->set_width(static_cast<uint32_t>(width));
    msg->mutable_info()->set_height(static_cast<uint32_t>(height));
    auto* origin = msg->mutable_info()->mutable_origin();
    origin->mutable_position()->set_x(min_ix * options_.voxel_size);
    origin->mutable_position()->set_y(min_iy * options_.voxel_size);
    origin->mutable_position()->set_z(0.0);
    origin->mutable_orientation()->set_w(1.0);
    msg->mutable_data()->Clear();
    msg->mutable_data()->Reserve(width * height);
    for (int8_t v : data) {
        msg->add_data(v);
    }
    return true;
}

}  // namespace map
}  // namespace atlas
}  // namespace localization
}  // namespace autonomy
