/*
 * Copyright 2026 The Openbot Authors
 *
 * Pluggable exploration backend interface and factory.
 */

#pragma once

#include <memory>
#include <string>
#include <vector>

#include <automsgs/msgs/geometry_msgs/polygon.pb.h>
#include <automsgs/msgs/geometry_msgs/pose_stamped.pb.h>
#include <automsgs/msgs/geometry_msgs/transform.pb.h>
#include <automsgs/msgs/map_msgs/occupancy_grid.pb.h>
#include <automsgs/msgs/nav_msgs/odometry.pb.h>
#include <automsgs/msgs/nav_msgs/path.pb.h>
#include <automsgs/msgs/sensor_msgs/camera_info.pb.h>
#include <automsgs/msgs/sensor_msgs/image.pb.h>
#include <automsgs/msgs/sensor_msgs/point_cloud2.pb.h>
#include <automsgs/msgs/visualization_msgs/marker_array.pb.h>

#include "autonomy/perception/proto/exploration_options.pb.h"

namespace autonomy::perception::exploration {

class ExplorerInterface {
 public:
  virtual ~ExplorerInterface() = default;

  virtual void Configure(const proto::ExplorationOptions& options) = 0;

  virtual void LoadBoundaries(
      const std::vector<std::string>& /*config_directories*/) {}

  virtual void UpdateOdometry(
      const automsgs::msgs::nav_msgs::Odometry& odom) = 0;

  virtual void UpdateDepth(
      const automsgs::msgs::sensor_msgs::Image& depth,
      const automsgs::msgs::sensor_msgs::CameraInfo& info,
      const automsgs::msgs::geometry_msgs::Transform& map_t_camera) = 0;

  virtual void UpdatePointCloud(
      const automsgs::msgs::sensor_msgs::PointCloud2& cloud) {}

  virtual void UpdateTerrainMap(
      const automsgs::msgs::sensor_msgs::PointCloud2& cloud) {}

  virtual void UpdatePriorMap(
      const automsgs::msgs::map_msgs::OccupancyGrid& map) {
    (void)map;
  }

  virtual void UpdatePlannerCostmap(
      const automsgs::msgs::map_msgs::OccupancyGrid& map) {
    (void)map;
  }

  virtual void Reset() {}

  virtual void SetPaused(bool paused) { paused_ = paused; }

  virtual bool IsPaused() const { return paused_; }

  virtual bool ExecutePlanningCycle() = 0;

  virtual bool HasTarget() const = 0;
  virtual bool GetNextWaypoint(
      automsgs::msgs::geometry_msgs::PoseStamped& waypoint) const = 0;
  virtual void MarkWaypointReached() = 0;

  virtual bool IsFinished() const = 0;
  virtual float Progress() const = 0;
  virtual float ExploredAreaM2() const = 0;

  virtual automsgs::msgs::nav_msgs::Path GetExplorationPath() const = 0;
  virtual automsgs::msgs::map_msgs::OccupancyGrid GetOccupancyGrid(
      const std::string& frame_id) const = 0;

  virtual automsgs::msgs::nav_msgs::Path GetGlobalDebugPath() const {
    return {};
  }
  virtual automsgs::msgs::nav_msgs::Path GetLocalDebugPath() const {
    return {};
  }

  virtual bool GetNavigationBoundary(
      automsgs::msgs::geometry_msgs::Polygon* boundary) const {
    (void)boundary;
    return false;
  }

  virtual automsgs::msgs::visualization_msgs::MarkerArray
  GetVisibilityGraphMarkers(const std::string& frame_id) const {
    (void)frame_id;
    return {};
  }

 protected:
  bool paused_{false};
};

class ExplorerFactory {
 public:
  static std::unique_ptr<ExplorerInterface> Create(
      const std::string& backend, const proto::ExplorationOptions& options);
};

}  // namespace autonomy::perception::exploration
