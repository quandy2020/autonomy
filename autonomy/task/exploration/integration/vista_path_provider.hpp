/* Copyright 2026 The Openbot Authors. */
#pragma once
#include "autonomy/task/exploration/integration/path_provider.hpp"
namespace autonomy::task::exploration {
class VistaPathProvider final : public PathProvider {
public:
    explicit VistaPathProvider(std::shared_ptr<planning::planner::vista::VistaRoutePlanner> planner);
    planning::planner::vista::VistaRoute Route(const Pose2d& start,const Pose2d& goal) const override;
    planning::planner::vista::VistaRoute Route(const Pose2d& start,const Pose2d& goal,
        const std::function<bool()>& canceled) const override;
    const std::shared_ptr<planning::planner::vista::VistaRoutePlanner>& planner() const{return planner_;}
    std::optional<RouteMapEvidence> Evidence() const override;
private:
    std::shared_ptr<planning::planner::vista::VistaRoutePlanner> planner_;
};
}  // namespace autonomy::task::exploration
