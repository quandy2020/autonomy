/* Copyright 2026 The Openbot Authors. */
#pragma once
#include "autonomy/task/exploration/observation/sensor_visibility_model.hpp"
namespace autonomy::task::exploration {
class CameraVisibilityModel final : public SensorVisibilityModel {
public:
    CameraVisibilityModel(perception::CameraModel camera,perception::RigidTransform base_from_camera);
    perception::SensorKind sensor() const override {return perception::SensorKind::kRgbd;}
    std::optional<CoverageContribution> Predict(const Pose2d&,const CoverageUnit&,
        const MapReadView&,const ObservationQuality&) const override;
private:
    perception::CameraModel camera_;
    perception::RigidTransform base_from_camera_;
};
}  // namespace autonomy::task::exploration
