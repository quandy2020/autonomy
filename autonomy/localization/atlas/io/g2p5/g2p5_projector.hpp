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

#pragma once

#include "autonomy/localization/atlas/type.hpp"

#include <vector>

namespace autonomy::localization::atlas {
namespace map {

/**
 * G2P5-style lidar → 2D occupancy side path (Lightning alignment).
 * Consumes Atlas-optimized poses; does not participate in dual-system fusion.
 */
class G2P5Projector {
public:
    struct Options {
        double resolution = 0.05;
        double min_z = -0.5;
        double max_z = 1.5;
        int width = 400;
        int height = 400;
    };

    G2P5Projector() = default;
    explicit G2P5Projector(Options options) : options_(std::move(options)) {}

    //! Project world points into a row-major occupancy buffer (0 free, 100 occ).
    std::vector<int8_t> Project(const std::vector<Vec3_t>& points_world) const;

    const Options& options() const { return options_; }

private:
    Options options_;
};

}  // namespace map
}  // namespace autonomy::localization::atlas
