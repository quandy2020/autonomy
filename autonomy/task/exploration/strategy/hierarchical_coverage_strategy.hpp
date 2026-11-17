/* Copyright 2026 The Openbot Authors. */
#pragma once
#include "autonomy/task/exploration/strategy/exploration_strategy.hpp"
#include "autonomy/task/exploration/frontier/frontier_queue.hpp"
#include "autonomy/task/exploration/viewpoint/viewpoint_sampler.hpp"
namespace autonomy::task::exploration {
struct HierarchicalCoverageOptions {
    double region_size=4,local_radius=5,fairness_interval=10;
    std::size_t local_views=5;
    perception::SensorMode mode=perception::SensorMode::kRgbdOnly;
    opt::navigation::SafetyOptions safety;
    ViewpointSamplingOptions sampling;
};
class HierarchicalCoverageStrategy final : public ExplorationStrategy {
public:
    HierarchicalCoverageStrategy(std::shared_ptr<const PathProvider> paths,
        std::shared_ptr<const ViewpointEvaluator> evaluator,HierarchicalCoverageOptions options={});
    void Reset() override;
    StrategyDecision Select(const Pose2d&,const Pose2d&,const MapReadView&,const CoverageState&,
        perception::SensorMask,double now,const std::function<bool()>& cancel_checker={}) override;
    FrontierQueue& frontier_queue(){return queue_;}
private:
    std::shared_ptr<const PathProvider> paths_;
    std::shared_ptr<const ViewpointEvaluator> evaluator_;
    HierarchicalCoverageOptions options_;
    RegionManager regions_;
    FrontierQueue queue_;
    std::uint64_t epoch_=0,next_goal_=0;
    double last_fair_selection_=-1e30;
    std::uint64_t sampling_map_revision_=0,sampling_coverage_revision_=0;
    perception::SensorMask sampling_sensors_=0;
    std::size_t normal_cursor_=0,sweep_cursor_=0,region_cursor_=0;
    bool sweep_completed_=false;
    std::uint64_t sweep_start_map_revision_=0,sweep_start_coverage_revision_=0;
    perception::SensorMask sweep_start_sensors_=0;
};
}  // namespace autonomy::task::exploration
