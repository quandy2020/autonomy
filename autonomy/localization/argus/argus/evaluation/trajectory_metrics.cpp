/*
 * Copyright 2026 The Openbot Authors
 */
#include "argus/evaluation/trajectory_metrics.hpp"

namespace argus {
namespace evaluation {

TrajectoryMetrics ComputeTrajectoryMetrics(
    const std::vector<TimedPose>& estimate,
    const std::vector<TimedPose>& ground_truth, double max_dt_sec) {
    return EvaluateTrajectories(estimate, ground_truth, max_dt_sec);
}

}  // namespace evaluation
}  // namespace argus
