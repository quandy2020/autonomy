/*
 * Copyright 2026 The Openbot Authors
 */

#pragma once

#include <vector>

#include <automsgs/msgs/geometry_msgs/point.pb.h>

#include "autonomy/perception/exploration/tare/lidar_model.hpp"
#include "autonomy/perception/exploration/common/planning_env.hpp"
#include "autonomy/perception/exploration/common/point_cloud_manager.hpp"
#include "autonomy/perception/proto/exploration_options.pb.h"
#include "autonomy/perception/exploration/tare/rolling_grid.hpp"
#include "autonomy/perception/exploration/common/terrain_height_map.hpp"
#include "autonomy/perception/exploration/common/types.hpp"

namespace autonomy::perception::exploration {

// Local 3D viewpoint lattice with rolling grid (TARE ViewpointManager).
class ViewpointManager {
 public:
  explicit ViewpointManager(const proto::ExplorationOptions& options);

  void SetOptions(const proto::ExplorationOptions& options);
  void SetTerrainMap(const TerrainHeightMap* terrain_map) {
    terrain_map_ = terrain_map;
  }
  void Update(const PlanningEnv& env, const PointCloudManager* cloud_manager,
              double robot_x, double robot_y, double robot_z);

  const std::vector<Viewpoint>& viewpoints() const { return viewpoints_; }
  const std::vector<Viewpoint>& selected() const { return selected_; }

  void SelectTopViewpoints(int max_count);
  double TotalGain() const;
  bool GetShortestPath(double from_x, double from_y, double to_x, double to_y,
                       std::vector<automsgs::msgs::geometry_msgs::Point>* path) const;

 private:
  void BuildLattice(double robot_x, double robot_y, double robot_z);
  void RollIfNeeded(double robot_x, double robot_y, double robot_z);
  void FilterViewpoints(const PlanningEnv& env);
  double ScoreViewpoint(const PlanningEnv& env, const PointCloudManager* cloud,
                        const Viewpoint& vp) const;
  int CountVisibleUnknown(const PlanningEnv& env, const Viewpoint& vp) const;

  proto::ExplorationOptions options_;
  const TerrainHeightMap* terrain_map_{nullptr};
  RollingGrid rolling_grid_;
  LidarModel lidar_model_;
  std::vector<Viewpoint> viewpoints_;
  std::vector<Viewpoint> selected_;
  double last_robot_x_{0.0};
  double last_robot_y_{0.0};
  double last_robot_z_{0.0};
  bool has_robot_{false};
};

}  // namespace autonomy::perception::exploration
