/* Copyright 2026 The Openbot Authors. */
/** @file hierarchical_coverage_strategy.cpp
 * @brief Hierarchical candidate selection with reusable compatible sweep evidence.
 */
#include "autonomy/task/exploration/strategy/hierarchical_coverage_strategy.hpp"
#include "autonomy/task/exploration/coverage/local_coverage_planner.hpp"
#include "autonomy/common/logging.hpp"
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
    if (!std::isfinite(options.sweep_max_age) || options.sweep_max_age <= 0)
        throw std::invalid_argument("invalid sweep proof lifetime");
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
    sweep_completed_at_ = 0;
    sweep_start_map_revision_ = sweep_start_coverage_revision_ = 0;
    sampling_start_cell_.reset();
    failed_viewpoints_.clear();
}
void HierarchicalCoverageStrategy::MarkFailed(const ObservationGoal& goal, double now) {
    failed_viewpoints_.push_back({goal.original_pose, now + 60.0});
    if (failed_viewpoints_.size() > 64) {
        failed_viewpoints_.erase(failed_viewpoints_.begin());
    }
}
StrategyDecision HierarchicalCoverageStrategy::Select(const Pose2d& start,const Pose2d& home,const MapReadView& map,
    const CoverageState& state,perception::SensorMask active,double now,
    const std::function<bool()>& cancel_checker){
    StrategyDecision decision;
    if (cancel_checker && cancel_checker()) { decision.canceled = true; return decision; }
    if(state.epoch()!=map.map_epoch || !active || !map.grid) {
        sweep_completed_ = false;
        return decision;
    }
    if(epoch_!=map.map_epoch){epoch_=map.map_epoch;queue_.Reset();last_fair_selection_=-1e30;
        sampling_map_revision_=0;region_cursor_=normal_cursor_=sweep_cursor_=0;sweep_completed_=false;
        sampling_start_cell_.reset();failed_viewpoints_.clear();}
    const auto cell = map.grid->WorldToCell(start);
    const auto origin = cell ? std::optional<std::pair<int, int>>{{cell->x, cell->y}} : std::nullopt;
    if(sampling_map_revision_!=map.geometry_revision ||
       sampling_coverage_revision_!=state.requirements_revision() ||
       sampling_sensors_!=active || sampling_start_cell_!=origin) {
        // Keep a bounded cursor moving, but do not accept a pass spanning new
        // work, changed geometry/capabilities or a different reachability seed.
        sweep_completed_=false;
        sampling_map_revision_=map.geometry_revision;
        sampling_coverage_revision_=state.requirements_revision();sampling_sensors_=active;
        // The sweep cursor is a pagination offset over map cells, not over the
        // current seed.  Resetting it when the robot crosses a cell boundary
        // can restart every page before a sparse free region is reached.
        sampling_start_cell_ = origin;
    }
    queue_.ReopenForMapChange(map.geometry_revision);
    queue_.Synchronize(state,now);
    regions_.Update(state);
    if (sweep_completed_ && (now < sweep_completed_at_ ||
                            now - sweep_completed_at_ > options_.sweep_max_age)) {
        sweep_completed_ = false;
        sweep_cursor_ = 0;
    }
    if (sweep_completed_ && !state.PendingCount()) {
        decision.grid_fallback_checked = decision.sweep_checked = true;
        return decision;
    }
    auto sampled=ViewpointSampler{}.Sample(start,map,state,*evaluator_,regions_,active,options_.safety,options_.sampling,
                                         false,normal_cursor_,cancel_checker);
    normal_cursor_=sampled.finished?0:sampled.next_cursor;
    if (sampled.canceled) { decision.canceled = true; return decision; }
    if(sampled.candidates.empty() || now-last_fair_selection_>=options_.fairness_interval){
        if(!sweep_cursor_) {
            sweep_start_map_revision_=map.geometry_revision;
            sweep_start_coverage_revision_=state.requirements_revision();sweep_start_sensors_=active;
        }
        auto swept=ViewpointSampler{}.Sample(start,map,state,*evaluator_,regions_,active,options_.safety,options_.sampling,
                                             true,sweep_cursor_,cancel_checker);
        if (swept.canceled) {
            sweep_cursor_ = swept.next_cursor;
            decision.canceled = true;
            return decision;
        }
        if(swept.finished){
            sweep_completed_=sweep_start_map_revision_==map.geometry_revision &&
                sweep_start_coverage_revision_==state.requirements_revision() && sweep_start_sensors_==active;
            if (sweep_completed_) sweep_completed_at_ = now;
            sweep_cursor_=0;
        }else sweep_cursor_=swept.next_cursor;
        sampled=std::move(swept);
    }
    if(sampled.canceled){decision.canceled=true;return decision;}
    sampled.candidates.erase(std::remove_if(sampled.candidates.begin(), sampled.candidates.end(),
        [&](const Viewpoint& candidate) {
            return std::any_of(failed_viewpoints_.begin(), failed_viewpoints_.end(),
                [&](const FailedViewpoint& failed) {
                    return now < failed.until &&
                        autonomy::navigation::Distance(candidate.pose, failed.pose) < 0.75;
                });
        }), sampled.candidates.end());
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
    for(const auto& candidate:sampled.candidates)if(autonomy::navigation::Distance(candidate.pose,start)<=options_.local_radius ||
        candidate.region_id==target_region || (mandatory && std::any_of(candidate.contributions.begin(),candidate.contributions.end(),
        [&](const auto& c){return c.id==*mandatory;})))local.push_back(candidate);
    auto selected=LocalCoveragePlanner{}.Select(local,state,options_.local_views,mandatory);
    if(selected.empty() && mandatory)selected=LocalCoveragePlanner{}.Select(local,state,options_.local_views);
    decision.local_route=VisitOrderOptimizer{}.Optimize(start,selected,*paths_,options_.sampling.nominal_speed,
                                                        options_.sampling.angular_speed,{},mandatory.has_value(),cancel_checker);
    AINFO_EVERY(5) << "EXPLORATION_SELECT candidates=" << sampled.candidates.size()
                   << " local=" << selected.size()
                   << " local_reachable=" << decision.local_route.reachable
                   << " selected_views=" << decision.local_route.views.size()
                   << " global_views=" << decision.global_route.views.size()
                   << " pending=" << state.PendingCount()
                   << " map_revision=" << map.geometry_revision
                   << " sampling_revision=" << sampling_map_revision_
                   << " sweep=" << sweep_completed_
                   << " limited=" << decision.sampling_limited
                   << " examined=" << sampled.examined
                   << " evaluated=" << sampled.evaluated
                   << " page_finished=" << sampled.finished;
    if (sampled.candidates.empty() && state.PendingCount()) {
        std::size_t frontiers = 0, surfaces = 0, directions = 0;
        double minimum_z = 1e30, maximum_z = -1e30;
        for (const auto& item : state.units()) {
            if (!item.second.pending) continue;
            frontiers += item.first.kind == CoverageKind::kFrontier;
            surfaces += item.first.kind == CoverageKind::kSurface;
            directions += item.first.kind == CoverageKind::kDirection;
            minimum_z = std::min(minimum_z, item.second.center.z);
            maximum_z = std::max(maximum_z, item.second.center.z);
        }
        AINFO_EVERY(5) << "EXPLORATION_PENDING frontiers=" << frontiers
                       << " surfaces=" << surfaces
                       << " directions=" << directions
                       << " z=[" << minimum_z << "," << maximum_z << "]"
                       << " candidates=" << sampled.candidates.size();
    }
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
