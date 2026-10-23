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

#include "argus/fusion/intensity_assist.hpp"

#include <algorithm>
#include <cmath>
#include <utility>

namespace argus {
namespace fusion {

IntensityAssist::IntensityAssist(IntensityAssistConfig config)
    : config_(std::move(config)) {}

double IntensityAssist::ComputeIntensityStdDev(
    const sensor::PointCloudXYZI& cloud) {
    if (cloud.empty()) {
        return 0.0;
    }
    double sum = 0.0;
    for (std::size_t i = 0; i < cloud.size(); ++i) {
        sum += static_cast<double>(cloud[i].intensity);
    }
    const double mean = sum / static_cast<double>(cloud.size());
    double variance = 0.0;
    for (std::size_t i = 0; i < cloud.size(); ++i) {
        const double delta =
            static_cast<double>(cloud[i].intensity) - mean;
        variance += delta * delta;
    }
    variance /= static_cast<double>(cloud.size());
    return std::sqrt(variance);
}

IntensityAssistResult IntensityAssist::Evaluate(
    const perception::DegeneracyResult& degeneracy,
    const sensor::PointCloudXYZI& cloud) const {
    IntensityAssistResult result;
    result.intensity_std_dev = ComputeIntensityStdDev(cloud);
    result.texture_score = std::clamp(
        result.intensity_std_dev /
            std::max(config_.min_intensity_std_dev, 1e-6),
        0.0, 1.0);
    result.available = config_.enable && degeneracy.is_degenerated &&
                       result.intensity_std_dev >=
                           config_.min_intensity_std_dev;
    if (result.available) {
        const double relief =
            std::clamp(config_.inflation_relief, 0.0, 1.0);
        result.inflation_multiplier =
            1.0 - relief * result.texture_score;
    }
    return result;
}

}  // namespace fusion
}  // namespace argus
