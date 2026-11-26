/* Copyright 2026 The Openbot Authors. */
/**
 * @file tare_explorer.hpp
 * @brief Middleware-free TARE-style hierarchical exploration planner.
 *
 * The upstream TARE package is a long-running ROS node rather than a
 * start/goal GlobalPlanner plugin. This native adapter therefore exposes an
 * exploration-cycle API and does not pretend to implement CreatePlan().
 */
#pragma once

#include <cstddef>
#include <functional>
#include <memory>
#include <optional>
#include <vector>

#ifndef TARE_EXPLORER_CORE_ONLY
#include <automsgs/msgs/geometry_msgs/pose_stamped.pb.h>
#include <automsgs/msgs/nav_msgs/path.pb.h>
#include "autonomy/planning/common/planner_interface.hpp"
#include "autonomy/planning/proto/planning_options.pb.h"
#endif

namespace autonomy::map::costmap_2d {
class Costmap2DWrapper;
}

namespace autonomy::planning::planner::tare_explorer {

/** Tunables corresponding to TARE global-cell and local-path selection. */
struct TareExplorerSettings {
    std::size_t min_frontier_cluster_size = 5;
    std::size_t max_global_viewpoints = 8;
    unsigned viewpoint_clearance_cells = 1;
    double information_gain_weight = 1.0;
    double travel_cost_weight = 1.0;
    bool allow_unknown_transit = false;
    std::size_t max_search_iterations = 1000000;
};

struct GridCell {
    unsigned x = 0;
    unsigned y = 0;
    friend bool operator==(const GridCell& lhs, const GridCell& rhs) {
        return lhs.x == rhs.x && lhs.y == rhs.y;
    }
    friend bool operator!=(const GridCell& lhs, const GridCell& rhs) {
        return !(lhs == rhs);
    }
};

/** Immutable Costmap snapshot consumed by one planning cycle. */
struct GridSnapshot {
    unsigned width = 0;
    unsigned height = 0;
    double resolution = 0.0;
    double origin_x = 0.0;
    double origin_y = 0.0;
    std::vector<unsigned char> costs;
};

/** Detailed outcome shared by strict routing and exploration-cycle callers. */
enum class TareExplorerStatus {
    kSuccess,
    kExplorationComplete,
    kCanceled,
    kInvalidMap,
    kInvalidStart,
    kInvalidGoal,
    kBlockedStart,
    kBlockedGoal,
    kNoPath,
    kSearchLimitExceeded,
};

/** One TARE cycle: ordered global viewpoints plus stitched local route. */
struct TareExplorerResult {
    std::vector<GridCell> path;
    std::vector<GridCell> viewpoints;
    TareExplorerStatus status = TareExplorerStatus::kNoPath;
    bool exploration_complete = false;
    bool return_home_failed = false;
    bool canceled = false;
};

/** Pure deterministic core, independent of ROS, PCL and OR-Tools. */
class TareExplorerCore {
public:
    explicit TareExplorerCore(TareExplorerSettings settings = {});

    /**
     * Select frontier subspaces and connect them with collision-free local paths.
     * If home is supplied, it is appended only after no reachable frontier
     * remains, matching TARE's exploration-complete return-home behavior.
     */
    TareExplorerResult Plan(
        const GridSnapshot& grid, GridCell robot,
        const std::optional<GridCell>& home = std::nullopt,
        const std::function<bool()>& cancel_checker = {}) const;

    /** Strict point-to-point route used only by the GlobalPlanner contract. */
    TareExplorerResult PlanRoute(
        const GridSnapshot& grid, GridCell start, GridCell goal,
        const std::function<bool()>& cancel_checker = {}) const;

private:
    TareExplorerSettings settings_;
};

#ifndef TARE_EXPLORER_CORE_ONLY
TareExplorerSettings ReadTareExplorerSettings(
    const proto::TareExplorerOptions& options);

/**
 * Native autonomy bridge. It snapshots the shared Costmap2D and converts the
 * pure core result to automsgs without owning a competing map or ROS node.
 */
class TareExplorerAdapter {
public:
    TareExplorerAdapter(
        std::shared_ptr<map::costmap_2d::Costmap2DWrapper> costmap,
        TareExplorerSettings settings = {});

    TareExplorerResult PlanExploration(
        const automsgs::msgs::geometry_msgs::PoseStamped& robot,
        const std::optional<automsgs::msgs::geometry_msgs::PoseStamped>& home,
        automsgs::msgs::nav_msgs::Path& path,
        const std::function<bool()>& cancel_checker = {}) const;

    TareExplorerResult PlanRoute(
        const automsgs::msgs::geometry_msgs::PoseStamped& start,
        const automsgs::msgs::geometry_msgs::PoseStamped& goal,
        automsgs::msgs::nav_msgs::Path& path,
        const std::function<bool()>& cancel_checker = {}) const;

private:
    std::shared_ptr<map::costmap_2d::Costmap2DWrapper> costmap_;
    TareExplorerCore core_;
};

/**
 * Plugin shell for consumers that require GlobalPlanner. CreatePlan remains a
 * strict start-to-goal route; exploration clients call CreateExplorationPlan.
 */
class TareExplorerPlanner final : public common::GlobalPlanner {
public:
    TareExplorerPlanner() = default;
    TareExplorerPlanner(
        const proto::PlannerOptions& options, const std::string& name,
        std::shared_ptr<map::costmap_2d::Costmap2DWrapper> costmap);

    uint32 CreatePlan(
        const automsgs::msgs::geometry_msgs::PoseStamped& start,
        const automsgs::msgs::geometry_msgs::PoseStamped& goal,
        automsgs::msgs::nav_msgs::Path& path,
        std::function<bool()> cancel_checker) override;

    TareExplorerResult CreateExplorationPlan(
        const automsgs::msgs::geometry_msgs::PoseStamped& robot,
        const std::optional<automsgs::msgs::geometry_msgs::PoseStamped>& home,
        automsgs::msgs::nav_msgs::Path& path,
        const std::function<bool()>& cancel_checker = {}) const;

private:
    std::shared_ptr<TareExplorerAdapter> explorer_;
};
#endif  // TARE_EXPLORER_CORE_ONLY

}  // namespace autonomy::planning::planner::tare_explorer
