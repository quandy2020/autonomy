/* Copyright 2026 The Openbot Authors. */
#pragma once
#include "autonomy/common/navigation/goal_adapter.hpp"
#include "autonomy/task/exploration/integration/path_provider.hpp"
#include "autonomy/task/exploration/viewpoint/viewpoint_evaluator.hpp"
namespace autonomy::task::exploration {
/** @brief Observation intent adapted to an executable pose and Vista route. */
struct AdaptedObservationGoal {
    ObservationGoal intent;   ///< Preserved measured-coverage requirements.
    Pose2d execution_pose;    ///< Exact final pose certified by route.
    double gain = 0;          ///< Predicted utility; never measured completion.
    double clearance = 0;     ///< Planar clearance at execution_pose.
    double route_length = 0;  ///< Full first-leg Vista route cost.
    planning::planner::vista::VistaRoute route;  ///< Immutable first-leg route.
};
class GoalPoseAdapter
{
public:
    GoalPoseAdapter(std::shared_ptr<const PathProvider> paths,
                    std::shared_ptr<const ViewpointEvaluator> evaluator,
                    autonomy::navigation::SafetyOptions safety = {},
                    double goal_clearance_reserve = 0);
    std::optional<AdaptedObservationGoal> Adapt(
        const ObservationGoal& goal, const Pose2d& start,
        const MapReadView& map, const CoverageState& coverage,
        perception::SensorMask active,
        const std::function<bool()>& cancel_checker = {}) const;

private:
    std::shared_ptr<const PathProvider> paths_;
    std::shared_ptr<const ViewpointEvaluator> evaluator_;
    autonomy::navigation::SafetyOptions safety_;
    double goal_clearance_reserve_ = 0;
};
}  // namespace autonomy::task::exploration
