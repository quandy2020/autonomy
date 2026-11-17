/* Copyright 2026 The Openbot Authors. */
#pragma once
#include <functional>
#include <mutex>
#include "autonomy/planning/planner/vista/graph_updater.hpp"
#include "opt/navigation/reachability.h"

namespace autonomy::planning::planner::vista {
struct VistaRoute {
    opt::navigation::Path path;
    double length = 0;
    std::uint64_t map_epoch = 0, geometry_revision = 0;
    bool success = false, used_grid_fallback = false, canceled = false;
    bool execution_verified = false;
};
class VistaRoutePlanner {
public:
    explicit VistaRoutePlanner(opt::navigation::SafetyOptions options = {},
                                std::size_t maximum_nodes = 4096);
    GraphUpdateStats Update(const perception::MapReadView& map);
    VistaRoute CreateRoute(const Pose2d& start, const Pose2d& goal,
                            const std::function<bool()>& canceled = {}) const;
    VisibilityGraph GraphSnapshot() const;
private:
    mutable std::mutex mutex_;
    opt::navigation::SafetyOptions options_;
    GraphUpdater updater_;
    VisibilityGraph graph_;
    std::optional<perception::MapReadView> map_;
};
}  // namespace autonomy::planning::planner::vista
