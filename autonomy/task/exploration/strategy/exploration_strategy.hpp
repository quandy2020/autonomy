/* Copyright 2026 The Openbot Authors. */
#pragma once
#include "autonomy/task/exploration/routing/visit_order_optimizer.hpp"
namespace autonomy::task::exploration {
struct StrategyDecision {
    ExplorationRoute local_route,global_route;
    std::optional<ObservationGoal> goal;
    bool grid_fallback_checked=false,sweep_checked=false,sampling_limited=false;
    bool canceled=false;
};
class ExplorationStrategy {
public:
    virtual ~ExplorationStrategy()=default;
    // Called on the strategy worker before the first job of every session.
    // Stateful implementations must clear cursors and selection/revisit history.
    virtual void Reset() {}
    virtual StrategyDecision Select(const Pose2d& start,const Pose2d& home,const MapReadView& map,
        const CoverageState& coverage,perception::SensorMask active,double now,
        const std::function<bool()>& cancel_checker={})=0;
};
}  // namespace autonomy::task::exploration
