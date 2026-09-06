/*
 * Copyright 2026 The Openbot Authors
 */

#pragma once

#include <string>

#include <automsgs/msgs/map_msgs/occupancy_grid.pb.h>
#include <automsgs/msgs/nav_msgs/path.pb.h>

#include "autonomy/perception/exploration/common/planning_utilities.hpp"
#include "autonomy/perception/exploration/common/planning_env.hpp"
#include "autonomy/perception/proto/exploration_options.pb.h"

namespace autonomy::perception::exploration {

// Debug overlays merged into published map/path (TAREVisualizer subset).
class ExplorationVisualizer {
 public:
  explicit ExplorationVisualizer(const proto::ExplorationOptions& options);

  void SetOptions(const proto::ExplorationOptions& options);

  automsgs::msgs::map_msgs::OccupancyGrid OverlayGrid(
      const PlanningEnv& env, const std::string& frame_id,
      const ExplorationPath& global_path,
      const ExplorationPath& local_path) const;

  automsgs::msgs::nav_msgs::Path MergePaths(
      const automsgs::msgs::nav_msgs::Path& global,
      const automsgs::msgs::nav_msgs::Path& local) const;

 private:
  proto::ExplorationOptions options_;
};

}  // namespace autonomy::perception::exploration
