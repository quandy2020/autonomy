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

/**
 * @file confidence_gating.hpp
 * @brief Unified observation admission. See docs/05_frontend_fusion.md.
 */

#ifndef ARGUS_FUSION_CONFIDENCE_GATING_HPP_
#define ARGUS_FUSION_CONFIDENCE_GATING_HPP_

#include <Eigen/Core>

#include "argus/perception/degeneracy_detector.hpp"
#include "argus/perception/sensor_health_monitor.hpp"

namespace argus {
namespace fusion {

/** @brief Thresholds for admitting LiDAR/visual observations into fusion. */
struct ConfidenceGatingConfig {
    /** @brief Floor on LiDAR fusion weight after gating. */
    double min_lidar_weight = 0.2;
    /** @brief Floor on visual fusion weight after gating. */
    double min_visual_weight = 0.1;
    /**
     * @brief Skip LiDAR when \(1/\kappa\) falls below this.
     *        Default 0.3 ≈ previous `skip_on_degeneracy_ratio` 0.7.
     */
    double min_inv_condition = 0.3;
    /** @brief Invalidate LiDAR when dynamic point fraction exceeds this. */
    double max_dynamic_ratio_lidar = 0.8;
    /** @brief Invalidate visual when dynamic fraction exceeds this. */
    double max_dynamic_ratio_visual = 0.5;
};

/** @brief Per-cycle admission weights and validity flags for LiDAR and vision.
 */
struct ObservationConfidence {
    /** @brief Effective LiDAR update weight in [0, 1]. */
    double lidar_weight = 1.0;
    /** @brief Effective visual update weight in [0, 1]. */
    double visual_weight = 1.0;
    /** @brief Weak observability directions from degeneracy analysis (LiDAR).
     */
    Eigen::Vector3d lidar_weak_directions = Eigen::Vector3d::Zero();
    /** @brief Whether LiDAR should be applied this cycle. */
    bool lidar_valid = true;
    /** @brief Whether visual should be applied this cycle. */
    bool visual_valid = true;
    /** @brief True when geometry is deemed degenerate for LiDAR. */
    bool is_degenerated = false;
};

/**
 * @brief Combines degeneracy, sensor health, and dynamic ratios into fusion
 * weights.
 * @see docs/05_frontend_fusion.md
 */
class ConfidenceGating
{
public:
    /** @param config Gating thresholds (defaults if omitted). */
    explicit ConfidenceGating(ConfidenceGatingConfig config = {});

    /**
     * @brief Compute observation confidence for one fusion cycle.
     * @param degeneracy LiDAR degeneracy / weak-direction estimate.
     * @param health Sensor health flags (dropout, timing, …).
     * @param dynamic_ratio_lidar Fraction of LiDAR points classified dynamic.
     * @param dynamic_ratio_visual Fraction of visual features classified
     * dynamic.
     * @return Weights, validity flags, and weak directions for downstream
     * fusion.
     */
    ObservationConfidence Evaluate(
        const perception::DegeneracyResult& degeneracy,
        const perception::SensorHealth& health, double dynamic_ratio_lidar,
        double dynamic_ratio_visual = 0.0) const;

    /** @return Active configuration (read-only). */
    const ConfidenceGatingConfig& config() const {
        return config_;
    }

private:
    ConfidenceGatingConfig config_;
};

}  // namespace fusion
}  // namespace argus

#endif  // ARGUS_FUSION_CONFIDENCE_GATING_HPP_
