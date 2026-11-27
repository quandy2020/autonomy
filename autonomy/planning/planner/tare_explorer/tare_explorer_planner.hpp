// Copyright 2026 The Openbot Authors.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
/**
 * @file tare_explorer_planner.hpp
 * @brief TARE-compatible planner id backed by the sole native Vista graph.
 */
#pragma once

#include <automsgs/msgs/geometry_msgs/pose_stamped.pb.h>
#include <automsgs/msgs/nav_msgs/path.pb.h>

#include <functional>
#include <memory>
#include <optional>
#include <string>

#include "autonomy/planning/common/planner_interface.hpp"
#include "autonomy/planning/planner/vista/vista_planner.hpp"
#include "autonomy/planning/proto/planning_options.pb.h"

namespace autonomy::map::costmap_2d {
class Costmap2DWrapper;
}  // namespace autonomy::map::costmap_2d

namespace autonomy::planning::planner::tare_explorer {

/**
 * @brief Compatibility planner id delegating every point-to-point route to
 * Vista.
 * @details Asynchronous exploration is owned by autonomy/task/exploration. This
 * class owns no frontier lifecycle, traversal graph, or route search.
 */
class TareExplorerPlanner final : public common::GlobalPlanner
{
public:
    /** @brief Constructs an unconfigured plugin instance. */
    TareExplorerPlanner() = default;

    /**
     * @brief Constructs a standalone compatibility id with one Vista owner.
     * @param options Planning server configuration.
     * @param name Plugin instance name.
     * @param costmap Shared autonomy costmap wrapper.
     * @details PlannerServer uses the shared-owner constructor below when both
     * Vista and TARE ids are installed.
     */
    TareExplorerPlanner(
        const proto::PlannerOptions& options, const std::string& name,
        std::shared_ptr<map::costmap_2d::Costmap2DWrapper> costmap);

    /**
     * @brief Constructs a configured compatibility id over an existing Vista
     * owner.
     * @param options Planning server configuration.
     * @param name Plugin instance name.
     * @param costmap Shared native Costmap owner.
     * @param vista Sole route-graph and map-evidence owner.
     * @throws std::invalid_argument If vista is null or uses another Costmap.
     */
    TareExplorerPlanner(
        const proto::PlannerOptions& options, const std::string& name,
        std::shared_ptr<map::costmap_2d::Costmap2DWrapper> costmap,
        std::shared_ptr<vista::VistaPlanner> vista);

    /** @return Sole Vista owner used by this compatibility id. */
    std::shared_ptr<vista::VistaPlanner> SharedVistaPlanner() const;

    /**
     * @brief Creates a strict start-to-goal route through Vista.
     * @param start Exact route start pose.
     * @param goal Exact route goal pose.
     * @param path Output route; cleared on failure or cancellation.
     * @param cancel_checker Cooperative cancellation callback.
     * @return A PlannerResultCode encoded as uint32.
     */
    uint32 CreatePlan(const automsgs::msgs::geometry_msgs::PoseStamped& start,
                      const automsgs::msgs::geometry_msgs::PoseStamped& goal,
                      automsgs::msgs::nav_msgs::Path& path,
                      std::function<bool()> cancel_checker) override;

    /**
     * @brief Rejects the retired planner-side exploration lifecycle.
     * @deprecated Use task::exploration::ExplorationSession.
     * @details Clears path, sets exploration_complete false, and returns
     * PLANNER_INVALID_PLUGIN without route work or invoking cancel_checker.
     */
    [[deprecated("Use task::exploration::ExplorationSession")]] uint32
    CreateExplorationPlan(
        const automsgs::msgs::geometry_msgs::PoseStamped& robot,
        const std::optional<automsgs::msgs::geometry_msgs::PoseStamped>& home,
        automsgs::msgs::nav_msgs::Path& path, bool* exploration_complete,
        const std::function<bool()>& cancel_checker = {}) const;

private:
    std::shared_ptr<vista::VistaPlanner> vista_;  ///< Sole route owner.
};

}  // namespace autonomy::planning::planner::tare_explorer
