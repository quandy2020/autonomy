/*
 * Copyright 2026 The Openbot Authors
 */

#pragma once

#include <cstdint>
#include <unordered_map>

#include <automsgs/msgs/sensor_msgs/point_cloud2.pb.h>

#include "autonomy/perception/proto/exploration_options.pb.h"

namespace autonomy::perception::exploration {

// Sparse ground-height grid (CMU /terrain_map subset for TARE viewpoint Z).
class TerrainHeightMap {
 public:
  explicit TerrainHeightMap(const proto::ExplorationOptions& options);

  void SetOptions(const proto::ExplorationOptions& options);
  void Clear();

  void UpdateFromPointCloud(
      const automsgs::msgs::sensor_msgs::PointCloud2& cloud, double robot_x,
      double robot_y, double robot_z);

  double QueryGroundZ(double x, double y, double fallback_z) const;

 private:
  static int64_t CellKey(int gx, int gy);
  void WorldToCell(double wx, double wy, int* gx, int* gy) const;

  proto::ExplorationOptions options_;
  std::unordered_map<int64_t, float> ground_z_;
};

}  // namespace autonomy::perception::exploration
