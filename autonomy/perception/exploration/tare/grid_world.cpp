/*
 * Copyright 2026 The Openbot Authors
 */

#include "autonomy/perception/exploration/tare/grid_world.hpp"

#include <algorithm>
#include <cmath>

#include "autonomy/perception/exploration/tare/viewpoint_manager.hpp"

namespace autonomy::perception::exploration {

GridWorld::GridWorld(const proto::ExplorationOptions& options) {
  SetOptions(options);
}

void GridWorld::SetOptions(const proto::ExplorationOptions& options) {
  options_ = options;
  cols_ = options.grid_cols() > 0 ? options.grid_cols() : 20;
  rows_ = options.grid_rows() > 0 ? options.grid_rows() : 20;
  cell_size_ = options.grid_cell_size_m() > 0 ? options.grid_cell_size_m() : 4.0;
  EnsureSize();
}

void GridWorld::SetOrigin(double x, double y) {
  origin_x_ = x - 0.5 * cols_ * cell_size_;
  origin_y_ = y - 0.5 * rows_ * cell_size_;
}

void GridWorld::EnsureSize() {
  status_.assign(static_cast<size_t>(cols_ * rows_), CellStatus::kUnseen);
  cell_gain_.assign(static_cast<size_t>(cols_ * rows_), 0.0);
}

int GridWorld::CellIndex(double x, double y) const {
  const int col = static_cast<int>(std::floor((x - origin_x_) / cell_size_));
  const int row = static_cast<int>(std::floor((y - origin_y_) / cell_size_));
  if (col < 0 || row < 0 || col >= cols_ || row >= rows_) {
    return -1;
  }
  return row * cols_ + col;
}

void GridWorld::UpdateCellStatus(const ViewpointManager& viewpoints,
                                 const PlanningEnv& env) {
  std::fill(cell_gain_.begin(), cell_gain_.end(), 0.0);
  for (const auto& vp : viewpoints.selected()) {
    const int idx = CellIndex(vp.x, vp.y);
    if (idx < 0) {
      continue;
    }
    cell_gain_[static_cast<size_t>(idx)] =
        std::max(cell_gain_[static_cast<size_t>(idx)], vp.score);
  }
  for (int i = 0; i < cols_ * rows_; ++i) {
    const auto center = CellCenter(i);
    if (!env.IsInExplorationArea(center.x(), center.y())) {
      status_[static_cast<size_t>(i)] = CellStatus::kCovered;
      continue;
    }
    if (cell_gain_[static_cast<size_t>(i)] > 0.0) {
      status_[static_cast<size_t>(i)] = CellStatus::kExploring;
    } else if (env.IsCovered(center.x(), center.y())) {
      status_[static_cast<size_t>(i)] = CellStatus::kCovered;
    }
  }
}

std::vector<int> GridWorld::GetExploringCells(double robot_x, double robot_y,
                                              int nearby_radius) const {
  std::vector<int> cells;
  const int robot_cell = CellIndex(robot_x, robot_y);
  const int robot_row = robot_cell >= 0 ? robot_cell / cols_ : -1;
  const int robot_col = robot_cell >= 0 ? robot_cell % cols_ : -1;
  for (int i = 0; i < cols_ * rows_; ++i) {
    if (status_[static_cast<size_t>(i)] != CellStatus::kExploring) {
      continue;
    }
    const int row = i / cols_;
    const int col = i % cols_;
    if (robot_row >= 0 &&
        std::abs(row - robot_row) <= nearby_radius &&
        std::abs(col - robot_col) <= nearby_radius) {
      continue;
    }
    cells.push_back(i);
  }
  return cells;
}

automsgs::msgs::geometry_msgs::Point GridWorld::CellCenter(
    int cell_index) const {
  automsgs::msgs::geometry_msgs::Point pt;
  const int row = cell_index / cols_;
  const int col = cell_index % cols_;
  pt.set_x(origin_x_ + (static_cast<double>(col) + 0.5) * cell_size_);
  pt.set_y(origin_y_ + (static_cast<double>(row) + 0.5) * cell_size_);
  return pt;
}

float GridWorld::CoverageProgress() const {
  int covered = 0;
  for (const auto status : status_) {
    if (status == CellStatus::kCovered) {
      ++covered;
    }
  }
  const int total = cols_ * rows_;
  return total > 0 ? static_cast<float>(covered) / static_cast<float>(total)
                     : 0.f;
}

void GridWorld::StoreCellPath(
    int from_cell, int to_cell,
    std::vector<automsgs::msgs::geometry_msgs::Point> path) {
  const int64_t key =
      (static_cast<int64_t>(from_cell) << 32) ^
      static_cast<int64_t>(to_cell & 0xffffffff);
  cell_paths_[key] = std::move(path);
}

const std::vector<automsgs::msgs::geometry_msgs::Point>*
GridWorld::GetCellPath(int from_cell, int to_cell) const {
  const int64_t key =
      (static_cast<int64_t>(from_cell) << 32) ^
      static_cast<int64_t>(to_cell & 0xffffffff);
  const auto it = cell_paths_.find(key);
  return it != cell_paths_.end() ? &it->second : nullptr;
}

void GridWorld::Reset() {
  status_.assign(status_.size(), CellStatus::kUnseen);
  cell_gain_.assign(cell_gain_.size(), 0.0);
  cell_paths_.clear();
}

}  // namespace autonomy::perception::exploration
