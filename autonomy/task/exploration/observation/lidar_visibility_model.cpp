/* Copyright 2026 The Openbot Authors. */
#include "autonomy/task/exploration/observation/lidar_visibility_model.hpp"
#include <stdexcept>
namespace autonomy::task::exploration {
LidarVisibilityModel::LidarVisibilityModel(std::vector<perception::Vec3> directions,double tolerance,
    double min_range,double max_range,perception::RigidTransform extrinsic)
    :directions_(std::move(directions)),minimum_cosine_(std::cos(tolerance)),
     minimum_range_(min_range),maximum_range_(max_range),base_from_lidar_(extrinsic) {
    if(directions_.empty() || !std::isfinite(tolerance) || tolerance<=0 || tolerance>1.57 ||
       !std::isfinite(min_range) || !std::isfinite(max_range) || min_range<=0 || max_range<=min_range ||
       !extrinsic.Valid())throw std::invalid_argument("invalid lidar visibility calibration");
    for(const auto& direction:directions_)
        if(!direction.Finite() || std::abs(direction.Norm()-1)>1e-5)
            throw std::invalid_argument("lidar beam directions must be normalized");
}
std::optional<CoverageContribution> LidarVisibilityModel::Predict(const Pose2d& base,const CoverageUnit& unit,
    const MapReadView& map,const ObservationQuality& quality) const {
    const auto pose=WorldFromSensor(base,base_from_lidar_);const auto d=unit.center-pose.translation;
    const double range=d.Norm();if(range<minimum_range_ || range>maximum_range_)return {};
    bool beam_visible=false;
    for(const auto& direction:directions_) {
        const auto world=pose.Rotate(direction);
        if((world.x*d.x+world.y*d.y+world.z*d.z)/range>=minimum_cosine_) {beam_visible=true;break;}
    }
    if(!beam_visible || !RayVisible(pose.translation,unit,map))return {};
    const double q=QualityWeight(unit,pose.translation,range,maximum_range_,quality);
    return q>0?std::optional<CoverageContribution>{{unit.id,sensor(),q}}:std::nullopt;
}
}  // namespace autonomy::task::exploration
