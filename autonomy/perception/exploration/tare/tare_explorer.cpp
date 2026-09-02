/*
 * Copyright 2026 The Openbot Authors
 */

#include "autonomy/perception/exploration/tare/tare_explorer.hpp"

#include "autonomy/perception/exploration/core/exploration_options.hpp"
#include "autonomy/perception/exploration/tare/tare_visualizer.hpp"

namespace autonomy::perception::exploration {

TareExplorer::TareExplorer() : TareExplorer(DefaultOptions()) {}

TareExplorer::TareExplorer(const proto::ExplorationOptions& options)
    : options_(options), planner_(options) {}

void TareExplorer::Configure(const proto::ExplorationOptions& options) {
  options_ = options;
  planner_.SetOptions(options);
}

void TareExplorer::LoadBoundaries(
    const std::vector<std::string>& config_directories) {
  planner_.LoadBoundaries(config_directories);
}

void TareExplorer::UpdateOdometry(
    const automsgs::msgs::nav_msgs::Odometry& odom) {
  planner_.UpdateOdometry(odom);
}

void TareExplorer::UpdateDepth(
    const automsgs::msgs::sensor_msgs::Image& depth,
    const automsgs::msgs::sensor_msgs::CameraInfo& info,
    const automsgs::msgs::geometry_msgs::Transform& map_t_camera) {
  planner_.UpdateDepth(depth, info, map_t_camera);
}

void TareExplorer::UpdatePointCloud(
    const automsgs::msgs::sensor_msgs::PointCloud2& cloud) {
  planner_.UpdatePointCloud(cloud);
}

void TareExplorer::UpdateTerrainMap(
    const automsgs::msgs::sensor_msgs::PointCloud2& cloud) {
  planner_.UpdateTerrainMap(cloud);
}

void TareExplorer::UpdatePriorMap(
    const automsgs::msgs::map_msgs::OccupancyGrid& map) {
  planner_.env().UpdatePriorMap(map);
}

void TareExplorer::UpdatePlannerCostmap(
    const automsgs::msgs::map_msgs::OccupancyGrid& map) {
  planner_.env().UpdatePlannerCostmap(map);
}

void TareExplorer::Reset() {
  planner_.Reset();
  paused_ = false;
}

bool TareExplorer::ExecutePlanningCycle() {
  if (paused_) {
    return false;
  }
  return planner_.ExecutePlanningCycle();
}

bool TareExplorer::HasTarget() const { return planner_.HasTarget(); }

bool TareExplorer::GetNextWaypoint(
    automsgs::msgs::geometry_msgs::PoseStamped& waypoint) const {
  if (!planner_.HasTarget()) {
    return false;
  }
  waypoint = planner_.GetLookahead();
  return true;
}

void TareExplorer::MarkWaypointReached() {
  planner_.AdvanceWaypointIndex();
}

bool TareExplorer::IsFinished() const { return planner_.IsFinished(); }

float TareExplorer::Progress() const { return planner_.Progress(); }

float TareExplorer::ExploredAreaM2() const {
  return planner_.ExploredAreaM2();
}

automsgs::msgs::nav_msgs::Path TareExplorer::GetExplorationPath() const {
  return planner_.path();
}

automsgs::msgs::map_msgs::OccupancyGrid TareExplorer::GetOccupancyGrid(
    const std::string& frame_id) const {
  const std::string id =
      frame_id.empty() ? options_.map_frame() : frame_id;
  return planner_.env().GetOccupancyGridWithOverlay(id, true);
}

automsgs::msgs::nav_msgs::Path TareExplorer::GetGlobalDebugPath() const {
  return planner_.global_debug_path();
}

automsgs::msgs::nav_msgs::Path TareExplorer::GetLocalDebugPath() const {
  return planner_.path();
}

bool TareExplorer::GetNavigationBoundary(
    automsgs::msgs::geometry_msgs::Polygon* boundary) const {
  if (boundary == nullptr || !planner_.env().HasExplorationBoundary()) {
    return false;
  }
  *boundary = planner_.env().GetExplorationBoundary();
  return true;
}

automsgs::msgs::visualization_msgs::MarkerArray
TareExplorer::GetVisibilityGraphMarkers(const std::string& frame_id) const {
  if (!options_.publish_vg_markers()) {
    return {};
  }
  const std::string id = frame_id.empty() ? options_.map_frame() : frame_id;
  return BuildTareGraphMarkers(planner_.grid_world(), planner_.keypose_graph(),
                               id);
}

}  // namespace autonomy::perception::exploration
