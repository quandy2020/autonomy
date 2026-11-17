/* Copyright 2026 The Openbot Authors. */
#pragma once
#include <optional>
#include "autonomy/task/exploration/coverage/coverage_state.hpp"
#include "autonomy/task/exploration/integration/map_reader.hpp"

namespace autonomy::task::exploration {
struct ObservationQuality {
    double valid_fraction = 1, localization_weight = 1, motion_weight = 1;
    double history_weight = 1;
};
class SensorVisibilityModel {
public:
    virtual ~SensorVisibilityModel() = default;
    virtual perception::SensorKind sensor() const = 0;
    virtual std::optional<CoverageContribution> Predict(const Pose2d& viewpoint,
        const CoverageUnit& unit, const MapReadView& map, const ObservationQuality& quality) const = 0;
};
// Sensor-local pose transformed through actual installation extrinsics and base yaw.
perception::RigidTransform WorldFromSensor(const Pose2d& base, const perception::RigidTransform& base_from_sensor);
bool RayVisible(const perception::Vec3& origin, const CoverageUnit& target, const MapReadView& map);
double QualityWeight(const CoverageUnit& unit,const perception::Vec3& origin,
                     double range, double maximum_range, const ObservationQuality& quality);
}  // namespace autonomy::task::exploration
