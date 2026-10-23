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

#include "argus/frontend/spatiotemporal_dynamic_filter.hpp"

#include <algorithm>
#include <cmath>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

namespace argus {
namespace frontend {

namespace {
constexpr double kDeg2Rad = M_PI / 180.0;
constexpr double kRad2Deg = 180.0 / M_PI;
}  // namespace

SpatiotemporalDynamicFilter::SpatiotemporalDynamicFilter(
    SpatiotemporalDynamicFilterConfig config)
    : config_(config) {
    const double az_span = 360.0;
    const double el_span =
        std::max(1.0, config_.elev_max_deg - config_.elev_min_deg);
    az_bins_ = std::max(
        1, static_cast<int>(std::ceil(az_span / config_.azimuth_res_deg)));
    el_bins_ = std::max(
        1, static_cast<int>(std::ceil(el_span / config_.elevation_res_deg)));
    prev_range_.assign(static_cast<std::size_t>(az_bins_ * el_bins_), 0.f);
    prev_valid_.assign(prev_range_.size(), 0);
}

void SpatiotemporalDynamicFilter::Reset() {
    std::fill(prev_range_.begin(), prev_range_.end(), 0.f);
    std::fill(prev_valid_.begin(), prev_valid_.end(), 0);
    has_prev_ = false;
}

SpatiotemporalDynamicFilter::SphericalIndex
SpatiotemporalDynamicFilter::Project(const Eigen::Vector3d& p) const {
    SphericalIndex idx;
    const double range = p.norm();
    if (range < config_.min_range_m || range > config_.max_range_m) {
        return idx;
    }
    const double az = std::atan2(p.y(), p.x()) * kRad2Deg;  // [-180, 180]
    const double el =
        std::asin(std::clamp(p.z() / range, -1.0, 1.0)) * kRad2Deg;
    if (el < config_.elev_min_deg || el > config_.elev_max_deg) {
        return idx;
    }
    double az_wrapped = az;
    if (az_wrapped < 0.0) {
        az_wrapped += 360.0;
    }
    idx.az = std::clamp(static_cast<int>(az_wrapped / config_.azimuth_res_deg),
                        0, az_bins_ - 1);
    idx.el = std::clamp(static_cast<int>((el - config_.elev_min_deg) /
                                         config_.elevation_res_deg),
                        0, el_bins_ - 1);
    idx.valid = true;
    return idx;
}

Mask SpatiotemporalDynamicFilter::FilterLidar(
    const sensor::PointCloudXYZI& cloud, const StatesGroup& /*state*/) {
    Mask mask;
    mask.keep.assign(cloud.size(), 1);

    std::vector<float> cur_range(prev_range_.size(), 0.f);
    std::vector<uint8_t> cur_valid(prev_range_.size(), 0);

    // Body / lidar frame after undistortion: scan-to-scan bearing compare.
    for (std::size_t i = 0; i < cloud.size(); ++i) {
        const Eigen::Vector3d p(cloud[i].x, cloud[i].y, cloud[i].z);
        const SphericalIndex idx = Project(p);
        if (!idx.valid) {
            continue;
        }
        const float range = static_cast<float>(p.norm());
        const int flat = FlatIndex(idx.az, idx.el);
        if (cur_valid[static_cast<std::size_t>(flat)] == 0 ||
            range < cur_range[static_cast<std::size_t>(flat)]) {
            cur_range[static_cast<std::size_t>(flat)] = range;
            cur_valid[static_cast<std::size_t>(flat)] = 1;
        }
    }

    if (has_prev_) {
        for (std::size_t i = 0; i < cloud.size(); ++i) {
            const Eigen::Vector3d p(cloud[i].x, cloud[i].y, cloud[i].z);
            const SphericalIndex idx = Project(p);
            if (!idx.valid) {
                continue;
            }
            const int flat = FlatIndex(idx.az, idx.el);
            const auto uflat = static_cast<std::size_t>(flat);
            if (prev_valid_[uflat] == 0) {
                continue;
            }
            const float cur = static_cast<float>(p.norm());
            const float prev = prev_range_[uflat];
            // Suddenly much closer along the same ray → dynamic intrusion.
            if (prev - cur > static_cast<float>(config_.range_diff_m)) {
                mask.keep[i] = 0;
            }
        }
    }

    prev_range_.swap(cur_range);
    prev_valid_.swap(cur_valid);
    has_prev_ = true;
    return mask;
}

Mask SpatiotemporalDynamicFilter::FilterVisual(const cv::Mat& image,
                                               const StatesGroup& /*state*/) {
    Mask m;
    if (!image.empty()) {
        m.keep.assign(static_cast<std::size_t>(image.rows * image.cols), 1);
    }
    return m;
}

}  // namespace frontend
}  // namespace argus
