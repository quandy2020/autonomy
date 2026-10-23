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
 * @file esikf_update_options.hpp
 * @brief Optional robustness hooks for one ESIKF LIO update.
 */

#ifndef ARGUS_BACKEND_STATE_FILTER_ESIKF_UPDATE_OPTIONS_HPP_
#define ARGUS_BACKEND_STATE_FILTER_ESIKF_UPDATE_OPTIONS_HPP_

#include "argus/fusion/confidence_gating.hpp"
#include "argus/fusion/intensity_assist.hpp"
#include "argus/perception/degeneracy_detector.hpp"
#include "argus/perception/sensor_health_monitor.hpp"

namespace argus {
namespace backend {

/** @brief Optional robustness hooks for one LIO update (default: off). */
struct EsikfUpdateOptions {
    EsikfUpdateOptions();

    /** @brief Run FIM degeneracy detection. */
    bool enable_degeneracy = false;
    perception::DegeneracyConfig degeneracy;
    /** @brief Optional degeneracy output. */
    perception::DegeneracyResult* degeneracy_result = nullptr;

    /** @brief Run observation confidence gating. */
    bool enable_gating = false;
    fusion::ConfidenceGatingConfig gating;
    /** @brief Current sensor health snapshot. */
    perception::SensorHealth health;
    /** @brief Dynamic point ratio (LiDAR). */
    double dynamic_ratio_lidar = 0.0;
    /** @brief Dynamic point ratio (visual). */
    double dynamic_ratio_visual = 0.0;
    /** @brief Optional gating output. */
    fusion::ObservationConfidence* observation_confidence = nullptr;

    /** @brief Soften degeneracy inflation when intensity texture is usable. */
    bool enable_intensity_assist = false;
    fusion::IntensityAssistConfig intensity_assist;
    /** @brief Optional intensity assist output. */
    fusion::IntensityAssistResult* intensity_assist_result = nullptr;
};

}  // namespace backend
}  // namespace argus

#endif  // ARGUS_BACKEND_STATE_FILTER_ESIKF_UPDATE_OPTIONS_HPP_
