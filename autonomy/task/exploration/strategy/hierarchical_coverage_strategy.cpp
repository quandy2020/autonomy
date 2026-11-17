/* Copyright 2026 The Openbot Authors. */
#include "autonomy/task/exploration/strategy/hierarchical_coverage_strategy.hpp"
#include "autonomy/task/exploration/coverage/local_coverage_planner.hpp"
#include <algorithm>
#include <stdexcept>
namespace autonomy::task::exploration {
HierarchicalCoverageStrategy::HierarchicalCoverageStrategy(std::shared_ptr<const PathProvider> paths,
    std::shared_ptr<const ViewpointEvaluator> evaluator,HierarchicalCoverageOptions options)
    :paths_(std::move(paths)),evaluator_(std::move(evaluator)),options_(options),regions_(options.region_size){
    if(!paths_ || !evaluator_ || !options.local_views || options.local_views>32 ||
       !std::isfinite(options.local_radius) || options.local_radius<=0 ||
       !std::isfinite(options.fairness_interval) || options.fairness_interval<=0)
        throw std::invalid_argument("invalid hierarchical coverage strategy");
}
void HierarchicalCoverageStrategy::Reset() {
    regions_ = RegionManager(options_.region_size);
    queue_.Reset();
    epoch_ = next_goal_ = 0;
    last_fair_selection_ = -1e30;
    sampling_map_revision_ = sampling_coverage_revision_ = 0;
    sampling_sensors_ = sweep_start_sensors_ = 0;
    normal_cursor_ = sweep_cursor_ = region_cursor_ = 0;
    sweep_completed_ = false;
    sweep_start_map_revision_ = sweep_start_coverage_revision_ = 0;
}
StrategyDecision HierarchicalCoverageStrategy::Select(const Pose2d& start,const Pose2d& home,const MapReadView& map,
    const CoverageState& state,perception::SensorMask active,double now,
    const std::function<bool()>& cancel_checker){
    StrategyDecision decision;if(state.epoch()!=map.map_epoch || !active)return decision;
    if(epoch_!=map.map_epoch){epoch_=map.map_epoch;queue_.Reset();last_fair_selection_=-1e30;
        sampling_map_revision_=0;region_cursor_=normal_cursor_=sweep_cursor_=0;sweep_completed_=false;}
    if(sampling_map_revision_!=map.geometry_revision || sampling_coverage_revision_!=state.revision() ||
       sampling_sensors_!=active) {
        // Preserve progress through pages during frequent updates. A completed
        // sweep is accepted only if its entire pass used one evidence revision.
        sweep_completed_=false;
        sampling_map_revision_=map.geometry_revision;sampling_coverage_revision_=state.revision();sampling_sensors_=active;
    }
    queue_.ReopenForMapChange(map.geometry_revision);
    queue_.Synchronize(state,now);
    regions_.Update(state);
    auto sampled=ViewpointSampler{}.Sample(start,map,state,*evaluator_,regions_,active,options_.safety,options_.sampling,
                                         false,normal_cursor_,cancel_checker);
    normal_cursor_=sampled.finished?0:sampled.next_cursor;
    if(sampled.candidates.empty() || now-last_fair_selection_>=options_.fairness_interval){
        if(!sweep_cursor_) {
            sweep_start_map_revision_=map.geometry_revision;
            sweep_start_coverage_revision_=state.revision();sweep_start_sensors_=active;
        }
        auto swept=ViewpointSampler{}.Sample(start,map,state,*evaluator_,regions_,active,options_.safety,options_.sampling,
                                             true,sweep_cursor_,cancel_checker);
        if(swept.finished){
            sweep_completed_=sweep_start_map_revision_==map.geometry_revision &&
                sweep_start_coverage_revision_==state.revision() && sweep_start_sensors_==active;
            sweep_cursor_=0;
        }else sweep_cursor_=swept.next_cursor;
        sampled=std::move(swept);
    }
    if(sampled.canceled){decision.canceled=true;return decision;}
    decision.sampling_limited=sampled.limited;decision.grid_fallback_checked=true;
    decision.sweep_checked=sweep_completed_;
    // Global coarse representative per region; local dense marginal-coverage route.
    std::map<std::pair<int,int>,Viewpoint> representatives;
    for(const auto& candidate:sampled.candidates){auto it=representatives.find(candidate.region_id);
        if(it==representatives.end() || candidate.Utility(candidate.predicted_gain)>it->second.Utility(it->second.predicted_gain))
            representatives[candidate.region_id]=candidate;
    }
    std::vector<Viewpoint> global;for(const auto& r:representatives)global.push_back(r.second);
    if(global.size()>128) {
        std::vector<Viewpoint> batch;
        for(std::size_t i=0;i<128;++i)batch.push_back(global[(region_cursor_+i)%global.size()]);
        region_cursor_=(region_cursor_+128)%global.size();global=std::move(batch);
        decision.sampling_limited=true;
    }
    decision.global_route=VisitOrderOptimizer{}.Optimize(start,global,*paths_,options_.sampling.nominal_speed,
                                                         options_.sampling.angular_speed,home,false,cancel_checker);
    std::optional<CoverageId> mandatory;
    if(now-last_fair_selection_>=options_.fairness_interval)mandatory=queue_.OldestDue(now);
    std::vector<Viewpoint> local;
    const auto target_region=decision.global_route.views.empty()?regions_.RegionFor(start):decision.global_route.views.front().region_id;
    for(const auto& candidate:sampled.candidates)if(opt::navigation::Distance(candidate.pose,start)<=options_.local_radius ||
        candidate.region_id==target_region || (mandatory && std::any_of(candidate.contributions.begin(),candidate.contributions.end(),
        [&](const auto& c){return c.id==*mandatory;})))local.push_back(candidate);
    auto selected=LocalCoveragePlanner{}.Select(local,state,options_.local_views,mandatory);
    if(selected.empty() && mandatory)selected=LocalCoveragePlanner{}.Select(local,state,options_.local_views);
    decision.local_route=VisitOrderOptimizer{}.Optimize(start,selected,*paths_,options_.sampling.nominal_speed,
                                                        options_.sampling.angular_speed,{},mandatory.has_value(),cancel_checker);
    if(cancel_checker && cancel_checker()){decision.canceled=true;return decision;}
    if(!decision.local_route.views.empty() && decision.local_route.reachable){
        const auto& view=decision.local_route.views.front();ObservationGoal goal;
        if(mandatory && std::any_of(view.contributions.begin(),view.contributions.end(),
            [&](const auto& c){return c.id==*mandatory && c.quality>0;}))last_fair_selection_=now;
        goal.goal_id=++next_goal_;goal.goal_version=1;goal.map_epoch=map.map_epoch;goal.coverage_revision=state.revision();
        goal.original_pose=view.pose;goal.region_id=view.region_id;goal.sensor_mode=options_.mode;
        for(const auto& c:view.contributions)if(c.quality>0 && std::find(goal.required_coverage_units.begin(),goal.required_coverage_units.end(),c.id)==goal.required_coverage_units.end())goal.required_coverage_units.push_back(c.id);
        goal.required_views={view.pose.yaw};goal.minimum_gain=view.predicted_gain*.5;decision.goal=std::move(goal);
    }
    return decision;
}
}  // namespace autonomy::task::exploration
