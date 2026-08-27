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

#include "autonomy/localization/atlas/map/occupancy_grid_map.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

#include "autonomy/localization/atlas/map/local_grid_maker.hpp"

namespace autonomy {
namespace localization {
namespace atlas {
namespace map {
namespace {

Eigen::Vector3f TransformCell(const Mat44_t& T, const Cell3D& c) {
    const Eigen::Vector4d h =
        T * Eigen::Vector4d(c.x, c.y, c.z, 1.0);
    return Eigen::Vector3f(static_cast<float>(h.x()), static_cast<float>(h.y()),
                           static_cast<float>(h.z()));
}

}  // namespace

OccupancyGridMap::OccupancyGridMap(Options options)
    : options_(std::move(options)) {
    if (options_.cell_size <= 1e-4f) {
        options_.cell_size = 0.05f;
    }
}

void OccupancyGridMap::Clear() {
    std::lock_guard<std::mutex> lock(mutex_);
    width_ = 0;
    height_ = 0;
    origin_x_ = 0.f;
    origin_y_ = 0.f;
    log_odds_.clear();
}

float OccupancyGridMap::LogOdds(float p) const {
    p = std::clamp(p, 1e-4f, 1.f - 1e-4f);
    return std::log(p / (1.f - p));
}

float OccupancyGridMap::ClampLogOdds(float l) const {
    const float lmin = LogOdds(options_.clamping_min);
    const float lmax = LogOdds(options_.clamping_max);
    return std::clamp(l, lmin, lmax);
}

float OccupancyGridMap::Prob(float log_odds) const {
    return 1.f - 1.f / (1.f + std::exp(log_odds));
}

int OccupancyGridMap::Index(int ix, int iy) const {
    return iy * width_ + ix;
}

void OccupancyGridMap::EnsureContains(float x, float y) {
    if (width_ == 0 || height_ == 0) {
        origin_x_ = x - options_.border_cells * options_.cell_size;
        origin_y_ = y - options_.border_cells * options_.cell_size;
        width_ = 1 + 2 * options_.border_cells;
        height_ = 1 + 2 * options_.border_cells;
        log_odds_.assign(static_cast<std::size_t>(width_ * height_),
                         std::numeric_limits<float>::quiet_NaN());
        return;
    }

    const float cell = options_.cell_size;
    int min_ix = static_cast<int>(std::floor((x - origin_x_) / cell));
    int min_iy = static_cast<int>(std::floor((y - origin_y_) / cell));
    int max_ix = min_ix;
    int max_iy = min_iy;

    int new_origin_ix = 0;
    int new_origin_iy = 0;
    int new_w = width_;
    int new_h = height_;
    bool grow = false;

    if (min_ix < 0) {
        new_origin_ix = min_ix - options_.border_cells;
        new_w = width_ - new_origin_ix;
        grow = true;
    }
    if (min_iy < 0) {
        new_origin_iy = min_iy - options_.border_cells;
        new_h = height_ - new_origin_iy;
        grow = true;
    }
    if (max_ix >= width_) {
        new_w = std::max(new_w, max_ix + 1 + options_.border_cells);
        grow = true;
    }
    if (max_iy >= height_) {
        new_h = std::max(new_h, max_iy + 1 + options_.border_cells);
        grow = true;
    }

    new_w = std::min(new_w, options_.max_width);
    new_h = std::min(new_h, options_.max_height);
    if (!grow || new_w <= 0 || new_h <= 0) {
        return;
    }

    std::vector<float> resized(
        static_cast<std::size_t>(new_w * new_h),
        std::numeric_limits<float>::quiet_NaN());
    const int offset_x = -new_origin_ix;
    const int offset_y = -new_origin_iy;
    for (int iy = 0; iy < height_; ++iy) {
        for (int ix = 0; ix < width_; ++ix) {
            const int nx = ix + offset_x;
            const int ny = iy + offset_y;
            if (nx < 0 || ny < 0 || nx >= new_w || ny >= new_h) {
                continue;
            }
            resized[static_cast<std::size_t>(ny * new_w + nx)] =
                log_odds_[static_cast<std::size_t>(Index(ix, iy))];
        }
    }
    origin_x_ += static_cast<float>(new_origin_ix) * cell;
    origin_y_ += static_cast<float>(new_origin_iy) * cell;
    width_ = new_w;
    height_ = new_h;
    log_odds_.swap(resized);
}

void OccupancyGridMap::Integrate(const LocalGrid& local) {
    if (local.empty_grid()) {
        return;
    }
    const Mat44_t T_map_link =
        OpencvPoseToRosCameraLink(local.T_wc_opencv);
    const float hit = LogOdds(options_.prob_hit);
    const float miss = LogOdds(options_.prob_miss);

    std::lock_guard<std::mutex> lock(mutex_);

    auto update = [&](const Cell3D& c, float delta) {
        const Eigen::Vector3f p = TransformCell(T_map_link, c);
        EnsureContains(p.x(), p.y());
        const int ix =
            static_cast<int>(std::floor((p.x() - origin_x_) / options_.cell_size));
        const int iy =
            static_cast<int>(std::floor((p.y() - origin_y_) / options_.cell_size));
        if (ix < 0 || iy < 0 || ix >= width_ || iy >= height_) {
            return;
        }
        float& cell = log_odds_[static_cast<std::size_t>(Index(ix, iy))];
        if (!std::isfinite(cell)) {
            cell = 0.f;
        }
        cell = ClampLogOdds(cell + delta);
    };

    for (const auto& c : local.empty) {
        update(c, miss);
    }
    for (const auto& c : local.ground) {
        update(c, miss);
    }
    for (const auto& c : local.obstacles) {
        update(c, hit);
    }
}

bool OccupancyGridMap::FillHeader(
    const std::string& frame_id, double timestamp_sec,
    automsgs::msgs::map_msgs::OccupancyGrid* msg) const {
    if (msg == nullptr || width_ <= 0 || height_ <= 0 || log_odds_.empty()) {
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
    *msg->mutable_info()->mutable_map_load_time() = msg->header().stamp();
    msg->mutable_info()->set_resolution(options_.cell_size);
    msg->mutable_info()->set_width(static_cast<uint32_t>(width_));
    msg->mutable_info()->set_height(static_cast<uint32_t>(height_));
    auto* origin = msg->mutable_info()->mutable_origin();
    origin->mutable_position()->set_x(origin_x_);
    origin->mutable_position()->set_y(origin_y_);
    origin->mutable_position()->set_z(0.0);
    origin->mutable_orientation()->set_w(1.0);
    origin->mutable_orientation()->set_x(0.0);
    origin->mutable_orientation()->set_y(0.0);
    origin->mutable_orientation()->set_z(0.0);
    return true;
}

bool OccupancyGridMap::IsOccupied(int ix, int iy) const {
    if (ix < 0 || iy < 0 || ix >= width_ || iy >= height_) {
        return false;
    }
    const float l = log_odds_[static_cast<std::size_t>(Index(ix, iy))];
    if (!std::isfinite(l)) {
        return false;
    }
    return Prob(l) >= options_.occupancy_thr;
}

bool OccupancyGridMap::ToMessage(
    const std::string& frame_id, double timestamp_sec,
    automsgs::msgs::map_msgs::OccupancyGrid* msg) const {
    if (msg == nullptr) {
        return false;
    }
    std::lock_guard<std::mutex> lock(mutex_);
    if (!FillHeader(frame_id, timestamp_sec, msg)) {
        return false;
    }

    msg->mutable_data()->Clear();
    msg->mutable_data()->Reserve(width_ * height_);
    const int erode = std::max(0, options_.erode_obstacles);
    for (int iy = 0; iy < height_; ++iy) {
        for (int ix = 0; ix < width_; ++ix) {
            const float l =
                log_odds_[static_cast<std::size_t>(Index(ix, iy))];
            if (!std::isfinite(l)) {
                msg->add_data(-1);
                continue;
            }
            bool occ = Prob(l) >= options_.occupancy_thr;
            // Erosion: occupied only if all neighbors in radius are occupied.
            if (occ && erode > 0) {
                for (int dy = -erode; dy <= erode && occ; ++dy) {
                    for (int dx = -erode; dx <= erode; ++dx) {
                        if (!IsOccupied(ix + dx, iy + dy)) {
                            occ = false;
                            break;
                        }
                    }
                }
            }
            msg->add_data(occ ? 100 : 0);
        }
    }
    return true;
}

bool OccupancyGridMap::ToProbMessage(
    const std::string& frame_id, double timestamp_sec,
    automsgs::msgs::map_msgs::OccupancyGrid* msg) const {
    if (msg == nullptr) {
        return false;
    }
    std::lock_guard<std::mutex> lock(mutex_);
    if (!FillHeader(frame_id, timestamp_sec, msg)) {
        return false;
    }
    msg->mutable_data()->Clear();
    msg->mutable_data()->Reserve(width_ * height_);
    for (int iy = 0; iy < height_; ++iy) {
        for (int ix = 0; ix < width_; ++ix) {
            const float l =
                log_odds_[static_cast<std::size_t>(Index(ix, iy))];
            if (!std::isfinite(l)) {
                msg->add_data(-1);
            } else {
                const int v = static_cast<int>(std::lround(Prob(l) * 100.f));
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
