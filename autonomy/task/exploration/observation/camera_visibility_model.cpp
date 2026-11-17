/* Copyright 2026 The Openbot Authors. */
#include "autonomy/task/exploration/observation/camera_visibility_model.hpp"
#include <stdexcept>
namespace autonomy::task::exploration {
CameraVisibilityModel::CameraVisibilityModel(perception::CameraModel model,perception::RigidTransform pose)
    :camera_(model),base_from_camera_(pose) {
    if(!model.Valid() || !pose.Valid())throw std::invalid_argument("invalid camera visibility calibration");
}
std::optional<CoverageContribution> CameraVisibilityModel::Predict(const Pose2d& base,const CoverageUnit& unit,
    const MapReadView& map,const ObservationQuality& quality) const {
    const auto pose=WorldFromSensor(base,base_from_camera_);const auto d=unit.center-pose.translation;
    const auto& r=pose.rotation;
    const perception::Vec3 optical{r[0]*d.x+r[3]*d.y+r[6]*d.z,
                                  r[1]*d.x+r[4]*d.y+r[7]*d.z,
                                  r[2]*d.x+r[5]*d.y+r[8]*d.z};
    if(optical.z<camera_.min_depth || optical.z>camera_.max_depth)return {};
    const double u=camera_.fx*optical.x/optical.z+camera_.cx,v=camera_.fy*optical.y/optical.z+camera_.cy;
    if(u<0 || u>=camera_.width || v<0 || v>=camera_.height || !RayVisible(pose.translation,unit,map))return {};
    const double q=QualityWeight(unit,pose.translation,d.Norm(),camera_.max_depth,quality);
    return q>0?std::optional<CoverageContribution>{{unit.id,sensor(),q}}:std::nullopt;
}
}  // namespace autonomy::task::exploration
