/* Copyright 2026 The Openbot Authors. */
/** @file viewpoint_evaluator.hpp
 * @brief Multi-sensor prediction from planar candidate viewpoints.
 */
#pragma once

#include <memory>
#include <vector>

#include "autonomy/task/exploration/observation/sensor_visibility_model.hpp"
#include "autonomy/task/exploration/viewpoint/viewpoint.hpp"

namespace autonomy::task::exploration {
/** @brief Predicts coverage using measured candidate-floor sensor elevation. */
class ViewpointEvaluator
{
public:
    /** @brief Creates an evaluator with at least one actual sensor model. */
    explicit ViewpointEvaluator(
        std::vector<std::shared_ptr<const SensorVisibilityModel>> models);

    /** @brief Predicts contributions from a planar candidate.
     *
     * Sensor elevation is relative to the finite measured floor height at the
     * candidate XY cell. The executable viewpoint and all route ownership
     * remain planar.
     */
    std::vector<CoverageContribution> Predict(
        const Pose2d& pose, const CoverageState& coverage,
        const MapReadView& map, perception::SensorMask active_sensors,
        const ObservationQuality& quality = {},
        bool include_completed = false) const;

private:
    std::vector<std::shared_ptr<const SensorVisibilityModel>> models_;
};
}  // namespace autonomy::task::exploration
