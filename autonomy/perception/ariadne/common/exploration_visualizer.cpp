/*
 * Copyright 2026 The Openbot Authors
 */

#include "autonomy/perception/exploration/common/exploration_visualizer.hpp"

namespace autonomy::perception::exploration {

ExplorationVisualizer::ExplorationVisualizer(
    const proto::ExplorationOptions& options) {
  SetOptions(options);
}

void ExplorationVisualizer::SetOptions(const proto::ExplorationOptions& options) {
  options_ = options;
}

automsgs::msgs::map_msgs::OccupancyGrid ExplorationVisualizer::OverlayGrid(
    const PlanningEnv& env, const std::string& frame_id,
    const ExplorationPath& global_path,
    const ExplorationPath& local_path) const {
  (void)global_path;
  (void)local_path;
  return env.GetOccupancyGridWithOverlay(frame_id, true);
}

automsgs::msgs::nav_msgs::Path ExplorationVisualizer::MergePaths(
    const automsgs::msgs::nav_msgs::Path& global,
    const automsgs::msgs::nav_msgs::Path& local) const {
  automsgs::msgs::nav_msgs::Path merged = global;
  for (const auto& pose : local.poses()) {
    *merged.add_poses() = pose;
  }
  return merged;
}

}  // namespace autonomy::perception::exploration
