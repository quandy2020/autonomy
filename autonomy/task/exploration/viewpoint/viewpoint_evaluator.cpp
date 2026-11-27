/* Copyright 2026 The Openbot Authors. */
/** @file viewpoint_evaluator.cpp
 * @brief Sensor coverage prediction from planar views and measured floor datum.
 */
#include "autonomy/task/exploration/viewpoint/viewpoint_evaluator.hpp"

#include <cmath>
#include <stdexcept>

namespace autonomy::task::exploration {
ViewpointEvaluator::ViewpointEvaluator(
    std::vector<std::shared_ptr<const SensorVisibilityModel>> models)
    : models_(std::move(models)) {
    if (models_.empty())
        throw std::invalid_argument(
            "exploration needs actual sensor visibility models");
    for (const auto& model : models_)
        if (!model)
            throw std::invalid_argument("null visibility model");
}

std::vector<CoverageContribution> ViewpointEvaluator::Predict(
    const Pose2d& pose, const CoverageState& coverage, const MapReadView& map,
    perception::SensorMask active, const ObservationQuality& quality,
    bool include_completed) const {
    std::vector<CoverageContribution> result;
    if (!autonomy::navigation::IsFinite(pose) ||
        coverage.epoch() != map.map_epoch || !map.grid)
        return result;
    const auto cell = map.grid->WorldToCell(pose);
    if (!cell)
        return result;
    const double floor_height = map.grid->At(cell->x, cell->y).floor_height;
    if (!std::isfinite(floor_height))
        return result;

    for (const auto& unit : coverage.units()) {
        if (!unit.second.pending && !include_completed)
            continue;
        // Sensor models intentionally retain a planar Pose2d API. Translating a
        // private prediction copy by the measured candidate floor is equivalent
        // to translating the installed sensor upward, without giving route
        // ownership or executable Z control to the viewpoint layer.
        auto floor_relative_unit = unit.second;
        floor_relative_unit.center.z -= floor_height;
        for (const auto& model : models_)
            if (active & perception::Mask(model->sensor())) {
                const auto index =
                    model->sensor() == perception::SensorKind::kLidar ? 0U : 1U;
                if (unit.second.required_sensors &&
                    (!(unit.second.required_sensors &
                       perception::Mask(model->sensor())) ||
                     (!include_completed &&
                      unit.second.observed_quality[index] >=
                          unit.second.required_quality)))
                    continue;
                auto prediction =
                    model->Predict(pose, floor_relative_unit, map, quality);
                if (prediction)
                    result.push_back(*prediction);
            }
    }
    return result;
}
}  // namespace autonomy::task::exploration
