#include <automsgs/msgs/geometry_msgs/polygon.pb.h>
#include <automsgs/msgs/sensor_msgs/point_cloud2.pb.h>/*
 * Copyright 2026 The Openbot Authors
 */

#pragma once

#include <cstdint>
#include <string>
#include <unordered_map>
#include <vector>

#include <automsgs/msgs/geometry_msgs/point.pb.h>
#include <automsgs/msgs/geometry_msgs/polygon.pb.h>
#include <automsgs/msgs/sensor_msgs/point_cloud2.pb.h>
#include <automsgs/msgs/map_msgs/occupancy_grid.pb.h>
#include <automsgs/msgs/nav_msgs/odometry.pb.h>
#include <automsgs/msgs/sensor_msgs/camera_info.pb.h>
#include <automsgs/msgs/sensor_msgs/image.pb.h>
#include <automsgs/msgs/geometry_msgs/transform.pb.h>

#include "autonomy/common/math/polygon2d.hpp"
#include "autonomy/map/costmap_2d/costmap_2d.hpp"
#include "autonomy/perception/proto/exploration_options.pb.h"

namespace autonomy::perception::exploration {

// Rolling 2D occupancy + coverage layer (TARE PlanningEnv, RGB-D adapted).
class PlanningEnv {
 public:
  explicit PlanningEnv(const proto::ExplorationOptions& options);

  void SetOptions(const proto::ExplorationOptions& options);
  void SetExplorationArea(
      const automsgs::msgs::geometry_msgs::Polygon& area);
  void SetNogoArea(const automsgs::msgs::geometry_msgs::Polygon& area);

  void UpdateOdometry(const automsgs::msgs::nav_msgs::Odometry& odom);
  void UpdateDepth(
      const automsgs::msgs::sensor_msgs::Image& depth,
      const automsgs::msgs::sensor_msgs::CameraInfo& info,
      const automsgs::msgs::geometry_msgs::Transform& map_t_camera);

  const std::vector<automsgs::msgs::geometry_msgs::Point>& frontiers() const {
    return frontiers_;
  }
  const std::vector<int>& frontier_cluster_sizes() const {
    return frontier_cluster_sizes_;
  }
  const map::costmap_2d::Costmap2D& costmap() const { return costmap_; }
  const map::costmap_2d::Costmap2D& inflated_costmap() const {
    return inflated_;
  }

  bool IsOccupied(double x, double y) const;
  bool IsFree(double x, double y) const;
  bool IsCovered(double x, double y) const;
  bool IsInExplorationArea(double x, double y) const;
  bool IsInNogoArea(double x, double y) const;

  void UpdatePointCloud(
      const automsgs::msgs::sensor_msgs::PointCloud2& cloud);

  void UpdatePriorMap(
      const automsgs::msgs::map_msgs::OccupancyGrid& map);

  void UpdatePlannerCostmap(
      const automsgs::msgs::map_msgs::OccupancyGrid& map);

  // Clear short-lived obstacles (dynamic objects) near world position.
  void ClearObstacleNear(double x, double y, double radius_m);

  double robot_x() const { return robot_x_; }
  double robot_y() const { return robot_y_; }
  double robot_z() const { return robot_z_; }
  double robot_yaw() const { return robot_yaw_; }

  int FrontierCellCount() const;
  bool HasExplorationBoundary() const;
  automsgs::msgs::geometry_msgs::Polygon GetExplorationBoundary() const;
  automsgs::msgs::map_msgs::OccupancyGrid GetOccupancyGrid(
      const std::string& frame_id) const;
  automsgs::msgs::map_msgs::OccupancyGrid GetOccupancyGridWithOverlay(
      const std::string& frame_id, bool mark_frontiers) const;

 private:
  void InitCostmap();
  void RollCostmapIfNeeded(double x, double y);
  void ApplyLogOdds(size_t index, float delta);
  void SyncCostmapFromLogOdds();
  void RebuildInflation();
  void ExtractFrontiers();
  void MarkCoveredFromExtrinsic(
      const automsgs::msgs::geometry_msgs::Transform& map_t_camera);
  void FuseDepthFrame(
      const automsgs::msgs::sensor_msgs::Image& depth,
      const automsgs::msgs::sensor_msgs::CameraInfo& info,
      const automsgs::msgs::geometry_msgs::Transform& map_t_camera);
  static int64_t GlobalKey(int gx, int gy);
  void WorldToGlobal(double wx, double wy, int* gx, int* gy) const;
  void PersistLocalToGlobal();
  void RestoreGlobalToLocal();
  void WriteGlobalLogOdds(double wx, double wy, float value);
  void WriteGlobalCovered(double wx, double wy, uint8_t value);
  bool ReadGlobalCovered(double wx, double wy) const;
  void FuseOccupancyGrid(const automsgs::msgs::map_msgs::OccupancyGrid& map,
                         int lethal_threshold, bool mark_free_unknown);

  proto::ExplorationOptions options_;
  map::costmap_2d::Costmap2D costmap_;
  map::costmap_2d::Costmap2D inflated_;
  ::autonomy::common::math::Polygon2d exploration_polygon_;
  ::autonomy::common::math::Polygon2d nogo_polygon_;

  std::vector<float> log_odds_;
  std::vector<uint8_t> covered_;
  std::unordered_map<int64_t, float> global_log_odds_;
  std::unordered_map<int64_t, uint8_t> global_covered_;
  std::vector<automsgs::msgs::geometry_msgs::Point> frontiers_;
  std::vector<int> frontier_cluster_sizes_;

  double robot_x_{0.0};
  double robot_y_{0.0};
  double robot_z_{0.0};
  double robot_yaw_{0.0};
  bool has_odom_{false};
};

}  // namespace autonomy::perception::exploration
