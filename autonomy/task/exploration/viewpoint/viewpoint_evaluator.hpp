/* Copyright 2026 The Openbot Authors. */
#pragma once
#include <memory>
#include "autonomy/task/exploration/observation/sensor_visibility_model.hpp"
#include "autonomy/task/exploration/viewpoint/viewpoint.hpp"
namespace autonomy::task::exploration {
class ViewpointEvaluator {
public:
    explicit ViewpointEvaluator(std::vector<std::shared_ptr<const SensorVisibilityModel>> models);
    std::vector<CoverageContribution> Predict(const Pose2d& pose,const CoverageState& coverage,
        const MapReadView& map,perception::SensorMask active_sensors,const ObservationQuality& quality={},
        bool include_completed=false) const;
private:
    std::vector<std::shared_ptr<const SensorVisibilityModel>> models_;
};
}  // namespace autonomy::task::exploration
