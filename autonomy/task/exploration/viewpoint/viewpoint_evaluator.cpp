/* Copyright 2026 The Openbot Authors. */
#include "autonomy/task/exploration/viewpoint/viewpoint_evaluator.hpp"
#include <stdexcept>
namespace autonomy::task::exploration {
ViewpointEvaluator::ViewpointEvaluator(std::vector<std::shared_ptr<const SensorVisibilityModel>> models)
    :models_(std::move(models)) {
    if(models_.empty())throw std::invalid_argument("exploration needs actual sensor visibility models");
    for(const auto& model:models_)if(!model)throw std::invalid_argument("null visibility model");
}
std::vector<CoverageContribution> ViewpointEvaluator::Predict(const Pose2d& pose,const CoverageState& coverage,
    const MapReadView& map,perception::SensorMask active,const ObservationQuality& quality,bool include_completed) const {
    std::vector<CoverageContribution> result;
    if(!autonomy::navigation::IsFinite(pose) || coverage.epoch()!=map.map_epoch)return result;
    for(const auto& unit:coverage.units()) {
        if(!unit.second.pending && !include_completed)continue;
        for(const auto& model:models_)if(active&perception::Mask(model->sensor())) {
            const auto index=model->sensor()==perception::SensorKind::kLidar?0U:1U;
            if(unit.second.required_sensors &&
               (!(unit.second.required_sensors&perception::Mask(model->sensor())) ||
                (!include_completed && unit.second.observed_quality[index]>=unit.second.required_quality)))continue;
            auto prediction=model->Predict(pose,unit.second,map,quality);
            if(prediction)result.push_back(*prediction);
        }
    }
    return result;
}
}  // namespace autonomy::task::exploration
