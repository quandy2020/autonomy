/* Copyright 2026 The Openbot Authors. */
#pragma once
#include "autonomy/task/exploration/observation/sensor_visibility_model.hpp"
namespace autonomy::task::exploration {
class LidarVisibilityModel final : public SensorVisibilityModel {
public:
    LidarVisibilityModel(std::vector<perception::Vec3> directions,double angular_tolerance,
                         double min_range,double max_range,perception::RigidTransform base_from_lidar={});
    perception::SensorKind sensor() const override {return perception::SensorKind::kLidar;}
    std::optional<CoverageContribution> Predict(const Pose2d&,const CoverageUnit&,
        const MapReadView&,const ObservationQuality&) const override;
private:
    std::vector<perception::Vec3> directions_;
    double minimum_cosine_,minimum_range_,maximum_range_;
    perception::RigidTransform base_from_lidar_;
};
}  // namespace autonomy::task::exploration
