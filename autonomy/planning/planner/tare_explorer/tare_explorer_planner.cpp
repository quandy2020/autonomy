// Copyright 2026 The Openbot Authors.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.

#include "autonomy/planning/planner/tare_explorer/tare_explorer_planner.hpp"

#include <cstdint>
#include <stdexcept>
#include <utility>

#include "autolink/plugin_manager/plugin_manager.hpp"

namespace autonomy::planning::planner::tare_explorer {

TareExplorerPlanner::TareExplorerPlanner(
    const proto::PlannerOptions& options, const std::string& name,
    std::shared_ptr<map::costmap_2d::Costmap2DWrapper> costmap)
    : GlobalPlanner(options, name, costmap),
      vista_(std::make_shared<vista::VistaPlanner>(options, name + ".vista",
                                                   std::move(costmap))) {}

TareExplorerPlanner::TareExplorerPlanner(
    const proto::PlannerOptions& options, const std::string& name,
    std::shared_ptr<map::costmap_2d::Costmap2DWrapper> costmap,
    std::shared_ptr<vista::VistaPlanner> vista)
    : GlobalPlanner(options, name, std::move(costmap)),
      vista_(std::move(vista)) {
    if (!vista_) {
        throw std::invalid_argument(
            "TARE compatibility requires a Vista owner");
    }
    if (vista_->GetCostmapWrapper() != GetCostmapWrapper()) {
        throw std::invalid_argument(
            "TARE compatibility and Vista require the same Costmap owner");
    }
}

std::shared_ptr<vista::VistaPlanner> TareExplorerPlanner::SharedVistaPlanner()
    const {
    return vista_;
}

uint32 TareExplorerPlanner::CreatePlan(
    const automsgs::msgs::geometry_msgs::PoseStamped& start,
    const automsgs::msgs::geometry_msgs::PoseStamped& goal,
    automsgs::msgs::nav_msgs::Path& path,
    std::function<bool()> cancel_checker) {
    path.Clear();
    if (!vista_) {
        return static_cast<uint32>(
            proto::PlannerResultCode::PLANNER_NOT_INITIALIZED);
    }
    return vista_->CreatePlan(start, goal, path, std::move(cancel_checker));
}

uint32 TareExplorerPlanner::CreateExplorationPlan(
    const automsgs::msgs::geometry_msgs::PoseStamped& robot,
    const std::optional<automsgs::msgs::geometry_msgs::PoseStamped>& home,
    automsgs::msgs::nav_msgs::Path& path, bool* exploration_complete,
    const std::function<bool()>& cancel_checker) const {
    static_cast<void>(robot);
    static_cast<void>(home);
    static_cast<void>(cancel_checker);
    path.Clear();
    if (exploration_complete) {
        *exploration_complete = false;
    }
    return static_cast<uint32>(
        proto::PlannerResultCode::PLANNER_INVALID_PLUGIN);
}

}  // namespace autonomy::planning::planner::tare_explorer

using autonomy::planning::common::GlobalPlanner;
using autonomy::planning::planner::tare_explorer::TareExplorerPlanner;
AUTOLINK_PLUGIN_MANAGER_REGISTER_PLUGIN(TareExplorerPlanner, GlobalPlanner);
