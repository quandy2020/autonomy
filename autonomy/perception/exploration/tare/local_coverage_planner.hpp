/*
 * Copyright 2026 The Openbot Authors
 */

#pragma once

#include <vector>

#include <automsgs/msgs/geometry_msgs/point.pb.h>
#include <automsgs/msgs/nav_msgs/path.pb.h>

#include "autonomy/perception/exploration/common/planning_env.hpp"
#include "autonomy/perception/proto/exploration_options.pb.h"
#include "autonomy/perception/exploration/tare/viewpoint_manager.hpp"

namespace autonomy::perception::exploration {

// Connects local viewpoints into a coverage path (TARE LocalCoveragePlanner).
class LocalCoveragePlanner {
 public:
  explicit LocalCoveragePlanner(const proto::ExplorationOptions& options);

  void SetOptions(const proto::ExplorationOptions& options);

  automsgs::msgs::nav_msgs::Path Solve(
      const PlanningEnv& env, const ViewpointManager& viewpoints,
      double robot_x, double robot_y,
      const std::vector<automsgs::msgs::geometry_msgs::Point>& global_targets);

 private:
  proto::ExplorationOptions options_;
};

}  // namespace autonomy::perception::exploration
