/*
 * Copyright 2026 The Openbot Authors
 */

#pragma once

#include <cstdint>
#include <unordered_map>
#include <vector>

#include <automsgs/msgs/sensor_msgs/point_cloud2.pb.h>

#include "autonomy/perception/proto/exploration_options.pb.h"

namespace autonomy::perception::exploration {

class PlanningEnv;

// Rolling 3D point storage for vertical-surface coverage (TARE PointCloudManager).
class PointCloudManager {
 public:
  explicit PointCloudManager(const proto::ExplorationOptions& options);

  void SetOptions(const proto::ExplorationOptions& options);
  void Clear();
  void Update(const automsgs::msgs::sensor_msgs::PointCloud2& cloud,
              double robot_x, double robot_y, double robot_z);

  const std::vector<double>& planner_x() const { return planner_x_; }
  const std::vector<double>& planner_y() const { return planner_y_; }
  const std::vector<double>& planner_z() const { return planner_z_; }
  const std::vector<double>& uncovered_x() const { return uncovered_x_; }
  const std::vector<double>& uncovered_y() const { return uncovered_y_; }
  const std::vector<double>& uncovered_z() const { return uncovered_z_; }

  void MarkCovered(double x, double y, double z, double radius_m);
  void RefreshUncovered(const PlanningEnv& env);

 private:
  static int64_t VoxelKey(int gx, int gy, int gz);
  void WorldToVoxel(double wx, double wy, double wz, int* gx, int* gy,
                    int* gz) const;

  proto::ExplorationOptions options_;
  std::unordered_map<int64_t, int> voxel_counts_;
  std::vector<double> planner_x_;
  std::vector<double> planner_y_;
  std::vector<double> planner_z_;
  std::vector<double> uncovered_x_;
  std::vector<double> uncovered_y_;
  std::vector<double> uncovered_z_;
  std::vector<uint8_t> covered_flags_;
};

}  // namespace autonomy::perception::exploration
