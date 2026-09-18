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

#include "autonomy/localization/atlas/io/g2p5/g2p5_projector.hpp"

#include <cmath>

namespace autonomy::localization::atlas {
namespace map {

G2P5Projector::G2P5Projector()
    : G2P5Projector(Options{}) {}

G2P5Projector::G2P5Projector(Options options)
    : options_(std::move(options)) {
    options_.g2p5.grid_map_resolution_ = options_.resolution;
    options_.g2p5.min_th_floor_ = options_.min_z;
    options_.g2p5.max_th_floor_ = options_.max_z;
    options_.g2p5.online_mode_ = false;
    g2p5_ = std::make_shared<G2P5>(options_.g2p5);
}

G2P5Projector::G2P5Projector(std::shared_ptr<G2P5> g2p5)
    : g2p5_(std::move(g2p5)) {
    if (g2p5_) {
        options_.g2p5 = g2p5_->options();
        options_.resolution = options_.g2p5.grid_map_resolution_;
        options_.min_z = options_.g2p5.min_th_floor_;
        options_.max_z = options_.g2p5.max_th_floor_;
    }
}

std::vector<int8_t> G2P5Projector::Project(
    const std::vector<Vec3_t>& points_world) const {
    return ProjectOnce(points_world, options_);
}

std::vector<int8_t> G2P5Projector::ProjectOnce(
    const std::vector<Vec3_t>& points_world, const Options& options) {
    // Prefer a lightweight fixed-size buffer for callers that only need
    // a flat occupancy image (LocalizationServer / debug). Full G2P5 ray
    // casting is available via G2P5::PushKeyframe.
    const int w = std::max(1, options.width);
    const int h = std::max(1, options.height);
    std::vector<int8_t> grid(static_cast<std::size_t>(w * h), 0);
    const double inv = 1.0 / std::max(1e-6, options.resolution);
    const double ox = 0.5 * w;
    const double oy = 0.5 * h;
    for (const auto& p : points_world) {
        if (!p.allFinite()) {
            continue;
        }
        if (p.z() < options.min_z || p.z() > options.max_z) {
            continue;
        }
        const int ix = static_cast<int>(std::floor(p.x() * inv + ox));
        const int iy = static_cast<int>(std::floor(p.y() * inv + oy));
        if (ix < 0 || iy < 0 || ix >= w || iy >= h) {
            continue;
        }
        grid[static_cast<std::size_t>(iy * w + ix)] = 100;
    }
    return grid;
}

}  // namespace map
}  // namespace autonomy::localization::atlas
