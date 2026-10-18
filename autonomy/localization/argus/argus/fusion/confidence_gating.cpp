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

#include "argus/fusion/confidence_gating.hpp"

#include <algorithm>
#include <cmath>
#include <utility>

namespace argus {
namespace fusion {

ConfidenceGating::ConfidenceGating(ConfidenceGatingConfig config)
    : config_(std::move(config)) {}

ObservationConfidence ConfidenceGating::Evaluate(
    const perception::DegeneracyResult& degeneracy,
    const perception::SensorHealth& health, double dynamic_ratio_lidar,
    double dynamic_ratio_visual) const {
    ObservationConfidence observation;
    observation.lidar_valid = health.lidar_healthy;
    observation.visual_valid = health.camera_healthy;
    observation.is_degenerated = degeneracy.is_degenerated;

    if (degeneracy.is_degenerated) {
        observation.lidar_weak_directions = degeneracy.weak_directions;
        const double inv_condition =
            1.0 / std::max(degeneracy.condition_number, 1.0);
        observation.lidar_weight =
            std::max(config_.min_lidar_weight, inv_condition);
        if (inv_condition < config_.min_inv_condition) {
            observation.lidar_valid = false;
        }
    }

    if (dynamic_ratio_lidar > config_.max_dynamic_ratio_lidar) {
        observation.lidar_weight =
            std::min(observation.lidar_weight, config_.min_lidar_weight);
    }
    if (dynamic_ratio_visual > config_.max_dynamic_ratio_visual) {
        observation.visual_weight = config_.min_visual_weight;
    }

    observation.visual_weight =
        std::max(observation.visual_weight, config_.min_visual_weight);
    if (observation.lidar_valid) {
        observation.lidar_weight =
            std::max(observation.lidar_weight, config_.min_lidar_weight);
    } else {
        observation.lidar_weight = 0.0;
    }
    if (!observation.visual_valid) {
        observation.visual_weight = 0.0;
    }
    return observation;
}

}  // namespace fusion
}  // namespace argus
