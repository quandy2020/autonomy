/*
 * Copyright 2026 The Openbot Authors
 */

#pragma once

#include <unordered_map>
#include <vector>

#include <automsgs/msgs/geometry_msgs/point.pb.h>

#include "autonomy/perception/exploration/common/types.hpp"
#include "autonomy/perception/exploration/common/planning_env.hpp"
#include "autonomy/perception/proto/exploration_options.pb.h"

namespace autonomy::perception::exploration {

class ViewpointManager;

// TARE coarse global decomposition (UNSEEN / EXPLORING / COVERED).
class GridWorld {
 public:
  explicit GridWorld(const proto::ExplorationOptions& options);

  void SetOptions(const proto::ExplorationOptions& options);
  void SetOrigin(double x, double y);
  void UpdateCellStatus(const ViewpointManager& viewpoints,
                        const PlanningEnv& env);

  const std::vector<CellStatus>& cell_status() const { return status_; }
  int cols() const { return cols_; }
  int rows() const { return rows_; }
  double cell_size() const { return cell_size_; }

  std::vector<int> GetExploringCells(double robot_x, double robot_y,
                                     int nearby_radius) const;
  automsgs::msgs::geometry_msgs::Point CellCenter(int cell_index) const;
  float CoverageProgress() const;

  void StoreCellPath(int from_cell, int to_cell,
                     std::vector<automsgs::msgs::geometry_msgs::Point> path);
  const std::vector<automsgs::msgs::geometry_msgs::Point>* GetCellPath(
      int from_cell, int to_cell) const;

  void Reset();

 private:
  int CellIndex(double x, double y) const;
  void EnsureSize();

  proto::ExplorationOptions options_;
  int cols_{20};
  int rows_{20};
  double cell_size_{4.0};
  double origin_x_{-40.0};
  double origin_y_{-40.0};
  std::vector<CellStatus> status_;
  std::vector<double> cell_gain_;
  std::unordered_map<int64_t, std::vector<automsgs::msgs::geometry_msgs::Point>>
      cell_paths_;
};

}  // namespace autonomy::perception::exploration
