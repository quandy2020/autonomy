/* Copyright 2026 The Openbot Authors. */
#pragma once
#include "autonomy/task/exploration/integration/path_provider.hpp"
#include "autonomy/task/exploration/viewpoint/viewpoint_evaluator.hpp"
#include "opt/navigation/goal_adapter.h"
namespace autonomy::task::exploration {
struct AdaptedObservationGoal {
    ObservationGoal intent;
    Pose2d execution_pose;
    double gain=0,clearance=0,route_length=0;
};
class GoalPoseAdapter {
public:
    GoalPoseAdapter(std::shared_ptr<const PathProvider> paths,
        std::shared_ptr<const ViewpointEvaluator> evaluator,opt::navigation::SafetyOptions safety={});
    std::optional<AdaptedObservationGoal> Adapt(const ObservationGoal& goal,const Pose2d& start,
        const MapReadView& map,const CoverageState& coverage,perception::SensorMask active,
        const std::function<bool()>& cancel_checker={}) const;
private:
    std::shared_ptr<const PathProvider> paths_;
    std::shared_ptr<const ViewpointEvaluator> evaluator_;
    opt::navigation::SafetyOptions safety_;
};
}  // namespace autonomy::task::exploration
