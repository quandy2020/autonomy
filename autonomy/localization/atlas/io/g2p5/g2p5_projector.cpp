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

std::vector<int8_t> G2P5Projector::Project(
    const std::vector<Vec3_t>& points_world) const {
    const int w = std::max(1, options_.width);
    const int h = std::max(1, options_.height);
    std::vector<int8_t> grid(static_cast<std::size_t>(w * h), 0);
    const double inv = 1.0 / std::max(1e-6, options_.resolution);
    const double ox = 0.5 * w;
    const double oy = 0.5 * h;
    for (const auto& p : points_world) {
        if (!p.allFinite()) {
            continue;
        }
        if (p.z() < options_.min_z || p.z() > options_.max_z) {
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
