/* Copyright 2026 The Openbot Authors. */
#pragma once
#include "autonomy/task/exploration/integration/path_provider.hpp"
#include "autonomy/task/exploration/viewpoint/viewpoint.hpp"
namespace autonomy::task::exploration {
struct ExplorationRoute {std::vector<Viewpoint> views;double distance=0,total_time=0;bool reachable=true;};
class VisitOrderOptimizer {
public:
    ExplorationRoute Optimize(const Pose2d& start,const std::vector<Viewpoint>& viewpoints,
        const PathProvider& paths,double nominal_speed,double angular_speed,
        const std::optional<Pose2d>& return_pose={},bool keep_first=false,
        const std::function<bool()>& cancel_checker={}) const;
};
}  // namespace autonomy::task::exploration
