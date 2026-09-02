/*
 * Copyright 2026 The Openbot Authors
 *
 * Full 3D FAR: contour polygons + persistent dynamic visibility graph.
 */

#pragma once

#include <optional>
#include <vector>

#include <automsgs/msgs/sensor_msgs/point_cloud2.pb.h>

#include "autonomy/perception/exploration/common/planning_env.hpp"
#include "autonomy/perception/exploration/common/planning_utilities.hpp"
#include "autonomy/perception/exploration/core/explorer.hpp"
#include "autonomy/perception/exploration/far3d/sensor_handlers3.hpp"
#include "autonomy/perception/exploration/far3d/visibility_graph3.hpp"

namespace autonomy::perception::exploration::far3d {

class Far3dExplorer final : public ExplorerInterface {
 public:
  Far3dExplorer();
  explicit Far3dExplorer(const proto::ExplorationOptions& options);

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

  const DynamicVisibilityGraph& visibility_graph() const {
    return visibility_graph_;
  }

 private:
  std::size_t SelectGoalNode() const;
  automsgs::msgs::geometry_msgs::PoseStamped ComputeLookahead() const;
  automsgs::msgs::geometry_msgs::PoseStamped MakePose(double x, double y,
                                                        double z,
                                                        double yaw) const;

  proto::ExplorationOptions options_;
  PlanningEnv env_;
  ContourDetector3 contour_detector_;
  ContourGraph3 contour_graph_;
  DynamicVisibilityGraph visibility_graph_;
  MapHandler3 map_handler_;
  ScanHandler3 scan_handler_;
  TerrainPlanner3 terrain_planner_;
  ViewpointExtension3 viewpoint_extension_;
  autonomy::perception::exploration::MomentumPlanner momentum_planner_;

  std::optional<automsgs::msgs::sensor_msgs::PointCloud2> latest_cloud_;
  double robot_z_{0.0};

  automsgs::msgs::nav_msgs::Path path_;
  automsgs::msgs::nav_msgs::Path visibility_debug_path_;
  automsgs::msgs::nav_msgs::Path previous_path_;
  automsgs::msgs::geometry_msgs::PoseStamped lookahead_;
  NavMode last_nav_mode_{NavMode::kAttemptable};
  bool has_target_{false};
  bool finished_{false};
  std::size_t waypoint_index_{0};
  std::string frame_id_{"map"};
};

}  // namespace autonomy::perception::exploration::far3d
