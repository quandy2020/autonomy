/*
 * Copyright 2026 The Openbot Authors
 *
 * FAR-style 2D visibility graph exploration (RGB-D mapping + VG planning).
 */

#pragma once

#include "autonomy/perception/exploration/core/explorer.hpp"
#include "autonomy/perception/exploration/far3d/sensor_handlers3.hpp"
#include "autonomy/perception/exploration/common/planning_utilities.hpp"
#include "autonomy/perception/exploration/common/planning_env.hpp"
#include "autonomy/perception/exploration/far2d/visibility_graph.hpp"
#include "autonomy/perception/exploration/far2d/visibility_graph.hpp"

#include <deque>
#include <utility>
#include <vector>

namespace autonomy::perception::exploration {

class FarExplorer final : public ExplorerInterface {
 public:
  FarExplorer();
  explicit FarExplorer(const proto::ExplorationOptions& options);

  void Configure(const proto::ExplorationOptions& options) override;
  void LoadBoundaries(
      const std::vector<std::string>& config_directories) override;

  void UpdateOdometry(
      const automsgs::msgs::nav_msgs::Odometry& odom) override;
  void UpdateDepth(
      const automsgs::msgs::sensor_msgs::Image& depth,
      const automsgs::msgs::sensor_msgs::CameraInfo& info,
      const automsgs::msgs::geometry_msgs::Transform& map_t_camera) override;

  void UpdatePointCloud(
      const automsgs::msgs::sensor_msgs::PointCloud2& cloud) override;

  void UpdatePriorMap(
      const automsgs::msgs::map_msgs::OccupancyGrid& map) override;

  void UpdatePlannerCostmap(
      const automsgs::msgs::map_msgs::OccupancyGrid& map) override;

  void Reset() override;

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

  automsgs::msgs::visualization_msgs::MarkerArray GetVisibilityGraphMarkers(
      const std::string& frame_id) const override;

  const PlanningEnv& env() const { return env_; }

 private:
  int SelectGoalNode() const;
  automsgs::msgs::geometry_msgs::PoseStamped ComputeLookahead() const;
  automsgs::msgs::geometry_msgs::PoseStamped MakePose(double x, double y,
                                                        double yaw) const;

  proto::ExplorationOptions options_;
  PlanningEnv env_;
  far3d::ScanHandler3 scan_handler_;
  VisibilityGraph visibility_graph_;
  MomentumPlanner momentum_planner_;
  automsgs::msgs::nav_msgs::Path path_;
  automsgs::msgs::nav_msgs::Path visibility_debug_path_;
  automsgs::msgs::nav_msgs::Path previous_path_;
  automsgs::msgs::geometry_msgs::PoseStamped lookahead_;
  bool has_target_{false};
  bool finished_{false};
  std::size_t waypoint_index_{0};
  std::deque<std::pair<double, double>> trajectory_;
  std::string frame_id_{"map"};
};

}  // namespace autonomy::perception::exploration
