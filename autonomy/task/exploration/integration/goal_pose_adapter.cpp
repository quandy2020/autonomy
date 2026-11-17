/* Copyright 2026 The Openbot Authors. */
#include "autonomy/task/exploration/integration/goal_pose_adapter.hpp"
#include "autonomy/task/exploration/coverage/coverage_fusion.hpp"
#include <algorithm>
#include <stdexcept>
namespace autonomy::task::exploration {
GoalPoseAdapter::GoalPoseAdapter(std::shared_ptr<const PathProvider> paths,
    std::shared_ptr<const ViewpointEvaluator> evaluator,opt::navigation::SafetyOptions safety)
    :paths_(std::move(paths)),evaluator_(std::move(evaluator)),safety_(safety){
    if(!paths_ || !evaluator_)throw std::invalid_argument("observation adapter requires route and visibility providers");
}
std::optional<AdaptedObservationGoal> GoalPoseAdapter::Adapt(const ObservationGoal& goal,const Pose2d& start,
    const MapReadView& map,const CoverageState& coverage,perception::SensorMask active,
    const std::function<bool()>& cancel_checker) const {
    if(!map.grid || goal.map_epoch!=map.map_epoch || coverage.epoch()!=map.map_epoch ||
       !opt::navigation::IsFinite(goal.original_pose) || !std::isfinite(goal.allowed_translation) ||
       goal.allowed_translation<0 || !std::isfinite(goal.allowed_yaw_change) || goal.allowed_yaw_change<0 ||
       !std::isfinite(goal.minimum_gain) || goal.minimum_gain<0 || !active)return {};
    if(goal.required_coverage_units.empty())return {};
    for(const auto& id:goal.required_coverage_units)if(!coverage.units().count(id))return {};
    for(double yaw:goal.required_views)if(!std::isfinite(yaw))return {};
    auto options=safety_;if(map.footprint_inflated){options.robot_radius=1e-9;options.horizontal_margin=0;}
    auto safety=std::make_shared<opt::navigation::GeometrySafety>(map.grid,options);
    std::optional<AdaptedObservationGoal> best;
    auto evaluate=[&](Pose2d pose){
        if(cancel_checker && cancel_checker())return;
        if(opt::navigation::Distance(pose,goal.original_pose)>goal.allowed_translation+1e-9 ||
           std::abs(opt::navigation::NormalizeAngle(pose.yaw-goal.original_pose.yaw))>goal.allowed_yaw_change+1e-9)return;
        const auto geometry=safety->VerifyPose(pose);if(!geometry.safe)return;
        auto adapted_intent=goal;
        if(adapted_intent.required_views.empty())adapted_intent.required_views={goal.original_pose.yaw};
        const double shift=opt::navigation::NormalizeAngle(pose.yaw-goal.original_pose.yaw);
        std::vector<CoverageContribution> predicted;
        for(auto& yaw:adapted_intent.required_views) {
            yaw=opt::navigation::NormalizeAngle(yaw+shift);
            auto view_pose=pose;view_pose.yaw=yaw;
            if(!safety->VerifyPose(view_pose).safe)return;
            const auto visible=evaluator_->Predict(view_pose,coverage,map,active,{},true);
            const auto union_coverage=CoverageFusion::Fuse(visible);
            for(const auto& id:goal.required_coverage_units)
                if(!union_coverage.count(id) || union_coverage.at(id)<=0)return;
            predicted.insert(predicted.end(),visible.begin(),visible.end());
        }
        const double gain=CoverageFusion::Gain(coverage,predicted);if(gain+1e-12<goal.minimum_gain)return;
        auto route=paths_->Route(start,pose,cancel_checker);if(!route.success || route.map_epoch!=map.map_epoch ||
           route.geometry_revision!=map.geometry_revision)return;
        const double score=gain/(route.length+0.1);
        if(!best || score>best->gain/(best->route_length+0.1))
            best=AdaptedObservationGoal{adapted_intent,pose,gain,geometry.clearance,route.length};
    };
    evaluate(goal.original_pose);
    for(double shift:{-goal.allowed_yaw_change,goal.allowed_yaw_change}) {
        auto pose=goal.original_pose;pose.yaw=opt::navigation::NormalizeAngle(pose.yaw+shift);evaluate(pose);
    }
    if(goal.allowed_translation<=0)return best;
    opt::navigation::AdaptationOptions tuning;
    tuning.fallback_radius=goal.allowed_translation;tuning.ring_max_radius=goal.allowed_translation;
    tuning.ring_min_radius=0;tuning.region_sample_spacing=std::min(map.grid->resolution(),.1);
    tuning.max_reachability_expansions=0;
    opt::navigation::AdaptationInput input;input.map=map.grid;input.safety=safety;input.robot_pose=start;
    input.goal.original=goal.original_pose;input.goal.frame_id=map.frame_id;input.region_only=true;
    input.cancel_checker=cancel_checker;
    input.goal.region=opt::navigation::GoalRegion{"observation_intent",goal.original_pose,goal.allowed_translation};
    const auto selected=opt::navigation::GoalAdapter(options,tuning).Adapt(input);
    for(const auto& candidate:selected.candidates)if(candidate.accepted){
        for(double yaw_delta:{0.0,-goal.allowed_yaw_change,goal.allowed_yaw_change}) {
            auto pose=candidate.pose;pose.yaw=opt::navigation::NormalizeAngle(goal.original_pose.yaw+yaw_delta);evaluate(pose);
        }
    }
    return best;
}
}  // namespace autonomy::task::exploration
