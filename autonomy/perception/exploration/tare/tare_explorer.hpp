/*
 * Copyright 2026 The Openbot Authors
 *
 * RGB-D autonomous exploration using TARE hierarchical planning.
 */

#pragma once

#include <memory>
#include <string>
#include <vector>

#include "autonomy/perception/exploration/core/explorer.hpp"
#include "autonomy/perception/exploration/tare/hierarchical_planner.hpp"

namespace autonomy::perception::exploration {

class TareExplorer final : public ExplorerInterface {
 public:
  TareExplorer();
  explicit TareExplorer(const proto::ExplorationOptions& options);

  void Configure(const proto::ExplorationOptions& options) override;
  void LoadBoundaries(const std::vector<std::string>& config_directories) override;

  void UpdateOdometry(
      const automsgs::msgs::nav_msgs::Odometry& odom) override;
  void UpdateDepth(
      const automsgs::msgs::sensor_msgs::Image& depth,
      const automsgs::msgs::sensor_msgs::CameraInfo& info,
      const automsgs::msgs::geometry_msgs::Transform& map_t_camera) override;

  void UpdatePointCloud(
      const automsgs::msgs::sensor_msgs::PointCloud2& cloud) override;

  void UpdateTerrainMap(
      const automsgs::msgs::sensor_msgs::PointCloud2& cloud) override;

  void UpdatePriorMap(
      const automsgs::msgs::map_msgs::OccupancyGrid& map) override;

  void UpdatePlannerCostmap(
      const automsgs::msgs::map_msgs::OccupancyGrid& map) override;

  void Reset() override;

  automsgs::msgs::visualization_msgs::MarkerArray GetVisibilityGraphMarkers(
      const std::string& frame_id) const override;

  bool ExecutePlanningCycle() override;
  bool HasTarget() const override;
  bool GetNextWaypoint(
      automsgs::msgs::geometry_msgs::PoseStamped& waypoint) const override;
  void MarkWaypointReached() override;

  bool IsFinished() const override;
  float Progress() const override;
  float ExploredAreaM2() const override;
  automsgs::msgs::nav_msgs::Path GetExplorationPath() const override;
  automsgs::msgs::map_msgs::OccupancyGrid GetOccupancyGrid(
      const std::string& frame_id) const override;
  automsgs::msgs::nav_msgs::Path GetGlobalDebugPath() const override;
  automsgs::msgs::nav_msgs::Path GetLocalDebugPath() const override;
  bool GetNavigationBoundary(
      automsgs::msgs::geometry_msgs::Polygon* boundary) const override;

 private:
  proto::ExplorationOptions options_;
  HierarchicalPlanner planner_;
};

}  // namespace autonomy::perception::exploration
