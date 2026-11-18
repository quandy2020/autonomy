/* Copyright 2026 The Openbot Authors. */
/** @file vista_path_provider.cpp
 * @brief Forward exploration queries to the shared Vista planner instance.
 */
#include "autonomy/task/exploration/integration/vista_path_provider.hpp"
#include <stdexcept>
namespace autonomy::task::exploration {
std::optional<RouteMapEvidence> VistaPathProvider::Evidence() const {
    const auto graph=planner_->GraphSnapshot();
    if(!graph.map_epoch)return {};
    return RouteMapEvidence{graph.map_epoch,graph.geometry_revision,
        graph.observation_watermark,graph.observation_cut,
        graph.observation_baseline,graph.observation_recovery_generation};
}
VistaPathProvider::VistaPathProvider(std::shared_ptr<planning::planner::vista::VistaRoutePlanner> planner)
    :planner_(std::move(planner)){if(!planner_)throw std::invalid_argument("Vista provider needs shared planner instance");}
planning::planner::vista::VistaRoute VistaPathProvider::Route(const Pose2d& start,const Pose2d& goal) const {
    return planner_->CreateRoute(start,goal);
}
planning::planner::vista::VistaRoute VistaPathProvider::Route(const Pose2d& start,const Pose2d& goal,
    const std::function<bool()>& canceled) const {
    return planner_->CreateRoute(start,goal,canceled);
}
}  // namespace autonomy::task::exploration
