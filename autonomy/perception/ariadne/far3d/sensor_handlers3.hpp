/*
 * Copyright 2026 The Openbot Authors
 *
 * FAR3D map, scan, terrain, and viewpoint-extension helpers.
 */

#pragma once

#include <cstdint>
#include <unordered_map>
#include <vector>

#include <automsgs/msgs/geometry_msgs/point.pb.h>
#include <automsgs/msgs/geometry_msgs/pose_stamped.pb.h>
#include <automsgs/msgs/sensor_msgs/point_cloud2.pb.h>

#include "autonomy/perception/exploration/far3d/types.hpp"
#include "autonomy/perception/proto/exploration_options.pb.h"

namespace autonomy::perception::exploration::far3d {

class MapHandler3 {
 public:
  explicit MapHandler3(const proto::ExplorationOptions& options);

  void SetOptions(const proto::ExplorationOptions& options);
  void Clear();
  void InsertObsCloud(const automsgs::msgs::sensor_msgs::PointCloud2& cloud);
  void InsertFreeCloud(const automsgs::msgs::sensor_msgs::PointCloud2& cloud);
  void RemoveObsNear(double x, double y, double z, double radius_m);

  bool HasObsBetween(double ax, double ay, double az, double bx, double by,
                     double bz) const;
  double TerrainHeightAt(double x, double y, double z_hint) const;

 private:
  static int64_t Key(int gx, int gy, int gz);
  void Voxel(double x, double y, double z, int* gx, int* gy, int* gz) const;

  proto::ExplorationOptions options_;
  std::unordered_map<int64_t, uint8_t> obs_cells_;
  std::unordered_map<int64_t, float> terrain_z_;
};

class ScanHandler3 {
 public:
  explicit ScanHandler3(const proto::ExplorationOptions& options);

  void SetOptions(const proto::ExplorationOptions& options);
  void ResetFrame();
  void MarkScan(const automsgs::msgs::sensor_msgs::PointCloud2& cloud,
                double robot_x, double robot_y, double robot_z);
  void MarkRay(double from_x, double from_y, double from_z, double to_x,
               double to_y, double to_z);
  void ExtractDynamicCloud(std::vector<double>* x, std::vector<double>* y,
                           std::vector<double>* z) const;

 private:
  enum CellBits : uint8_t { kEmpty = 0, kScan = 1, kRay = 2, kObs = 4 };

  static int64_t Key(int gx, int gy, int gz);
  void Voxel(double x, double y, double z, int* gx, int* gy, int* gz) const;

  proto::ExplorationOptions options_;
  std::unordered_map<int64_t, uint8_t> cells_;
};

class TerrainPlanner3 {
 public:
  explicit TerrainPlanner3(const proto::ExplorationOptions& options);

  void SetOptions(const proto::ExplorationOptions& options);
  void SetCenter(double x, double y);
  void SetObstacles(const MapHandler3& map);

  bool Plan(double from_x, double from_y, double to_x, double to_y,
            std::vector<automsgs::msgs::geometry_msgs::Point>* path) const;
  bool IsOccupied(double x, double y) const;

 private:
  proto::ExplorationOptions options_;
  double center_x_{0.0};
  double center_y_{0.0};
  int grid_size_{40};
  std::vector<uint8_t> grid_;
};

class ViewpointExtension3 {
 public:
  explicit ViewpointExtension3(const proto::ExplorationOptions& options);

  void SetOptions(const proto::ExplorationOptions& options);

  automsgs::msgs::geometry_msgs::PoseStamped Extend(
      const automsgs::msgs::geometry_msgs::PoseStamped& waypoint,
      const CtNode& corner, const MapHandler3& map) const;

 private:
  proto::ExplorationOptions options_;
};

}  // namespace autonomy::perception::exploration::far3d
