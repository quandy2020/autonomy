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
 * @file intensity_assist.hpp
 * @brief Intensity texture gate for degeneracy: soften P inflation when useful.
 */

#ifndef ARGUS_FUSION_INTENSITY_ASSIST_HPP_
#define ARGUS_FUSION_INTENSITY_ASSIST_HPP_

#include "argus/perception/degeneracy_detector.hpp"
#include "argus/sensor/point_types.hpp"

namespace argus {
namespace fusion {

struct IntensityAssistConfig {
    bool enable = false;
    /** @brief Min intensity std-dev (raw intensity units) to treat as textured.
     */
    double min_intensity_std_dev = 8.0;
    /** @brief When textured, multiply degeneracy inflation by this (0..1). */
    double inflation_relief = 0.35;
    /** @brief Soft information scale reserved for future intensity residuals.
     */
    double information_scale = 20.0;
};

struct IntensityAssistResult {
    bool available = false;
    double texture_score = 0.0;  // [0,1]
    double intensity_std_dev = 0.0;
    /** @brief Multiply anisotropic inflation α by this when available. */
    double inflation_multiplier = 1.0;
};

class IntensityAssist
{
public:
    /**
     * @brief Construct with texture gate thresholds.
     * @param config Enable flag, std-dev gate, and inflation relief.
     */
    explicit IntensityAssist(IntensityAssistConfig config = {});

    /**
     * @brief Population std-dev of point intensities; 0 if empty.
     * @param cloud LiDAR scan with intensity channel.
     * @return Standard deviation of intensities, or 0 when empty.
     */
    static double ComputeIntensityStdDev(const sensor::PointCloudXYZI& cloud);

    /**
     * @brief Score texture and scale degeneracy inflation when configured.
     * @param degeneracy Latest degeneracy detector output.
     * @param cloud Scan used for intensity statistics.
     * @return Assist result with inflation_multiplier for fusion.
     */
    IntensityAssistResult Evaluate(
        const perception::DegeneracyResult& degeneracy,
        const sensor::PointCloudXYZI& cloud) const;

    /** @return Active configuration. */
    const IntensityAssistConfig& config() const {
        return config_;
    }

private:
    IntensityAssistConfig config_;
};

}  // namespace fusion
}  // namespace argus

#endif  // ARGUS_FUSION_INTENSITY_ASSIST_HPP_
