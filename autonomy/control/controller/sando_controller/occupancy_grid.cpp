/*
 * Copyright 2025 The Openbot Authors (duyongquan)
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

/**
 * @file occupancy_grid.cpp
 * @brief Costmap rasterization, inflation, heat field, and point-cloud extraction.
 *
 * Declarations and the algorithm contract live in the matching header.
 * This file holds the definitions.
 */

#include "autonomy/control/controller/sando_controller/occupancy_grid.hpp"

#include <algorithm>
#include <cmath>

#include "Eigen/Dense"
#include "autonomy/map/costmap_2d/cost_values.hpp"

namespace autonomy {
namespace control {
namespace controller {
namespace sando_controller {

void OccupancyGrid::LoadFromCostmap(const map::costmap_2d::Costmap2D& costmap, double inflation_radius,
                         double focus_x, double focus_y, double focus_radius) {
  width_ = static_cast<int>(costmap.getSizeInCellsX());
  height_ = static_cast<int>(costmap.getSizeInCellsY());
  resolution_ = costmap.getResolution();
  origin_x_ = costmap.getOriginX();
  origin_y_ = costmap.getOriginY();
  cells_.assign(static_cast<size_t>(width_ * height_), static_cast<uint8_t>(Cell::kFree));
  heat_.assign(cells_.size(), 0.0f);
  if (width_ <= 0 || height_ <= 0 || resolution_ <= 1e-6) {
    return;
  }

  for (int row_index = 0; row_index < height_; ++row_index) {
    for (int column_index = 0; column_index < width_; ++column_index) {
      const unsigned char cost = costmap.getCost(static_cast<unsigned int>(column_index),
                                                  static_cast<unsigned int>(row_index));
      Cell cell = Cell::kFree;
      if (cost == map::costmap_2d::NO_INFORMATION) {
        cell = Cell::kUnknown;
      } else if (cost >= map::costmap_2d::INSCRIBED_INFLATED_OBSTACLE) {
        cell = Cell::kOccupied;
      }
      cells_[static_cast<size_t>(ComputeLinearIndex(column_index, row_index))] = static_cast<uint8_t>(cell);
    }
  }

  if (inflation_radius <= resolution_) {
    return;
  }
  const int rad = static_cast<int>(std::ceil(inflation_radius / resolution_));
  const double r2 = inflation_radius * inflation_radius;
  std::vector<uint8_t> inflated = cells_;
  for (int row_index = 0; row_index < height_; ++row_index) {
    for (int column_index = 0; column_index < width_; ++column_index) {
      if (cells_[static_cast<size_t>(ComputeLinearIndex(column_index, row_index))] != static_cast<uint8_t>(Cell::kOccupied)) {
        continue;
      }
      if (focus_radius > 0.0) {
        double world_x = 0.0;
        double world_y = 0.0;
        ConvertCellToWorld(column_index, row_index, &world_x, &world_y);
        if (std::hypot(world_x - focus_x, world_y - focus_y) > focus_radius) {
          continue;
        }
      }
      for (int dy = -rad; dy <= rad; ++dy) {
        for (int dx = -rad; dx <= rad; ++dx) {
          const int nx = column_index + dx;
          const int ny = row_index + dy;
          if (nx < 0 || ny < 0 || nx >= width_ || ny >= height_) {
            continue;
          }
          if ((dx * dx + dy * dy) * resolution_ * resolution_ > r2) {
            continue;
          }
          auto& dst = inflated[static_cast<size_t>(ComputeLinearIndex(nx, ny))];
          if (dst != static_cast<uint8_t>(Cell::kOccupied)) {
            dst = static_cast<uint8_t>(Cell::kOccupied);
          }
        }
      }
    }
  }
  cells_.swap(inflated);
}

bool OccupancyGrid::ConvertWorldToCell(double x, double y, int* column_index, int* row_index) const {
  if (IsEmpty()) {
    return false;
  }
  const int map_column = static_cast<int>(std::floor((x - origin_x_) / resolution_));
  const int map_row = static_cast<int>(std::floor((y - origin_y_) / resolution_));
  if (map_column < 0 || map_row < 0 || map_column >= width_ || map_row >= height_) {
    return false;
  }
  *column_index = map_column;
  *row_index = map_row;
  return true;
}

void OccupancyGrid::ConvertCellToWorld(int column_index, int row_index, double* x, double* y) const {
  *x = origin_x_ + (static_cast<double>(column_index) + 0.5) * resolution_;
  *y = origin_y_ + (static_cast<double>(row_index) + 0.5) * resolution_;
}

Cell OccupancyGrid::GetCell(int column_index, int row_index) const {
  if (column_index < 0 || row_index < 0 || column_index >= width_ || row_index >= height_) {
    return Cell::kOccupied;
  }
  return static_cast<Cell>(cells_[static_cast<size_t>(ComputeLinearIndex(column_index, row_index))]);
}

float OccupancyGrid::GetCellHeat(int column_index, int row_index) const {
  if (column_index < 0 || row_index < 0 || column_index >= width_ || row_index >= height_) {
    return 0.0f;
  }
  return heat_[static_cast<size_t>(ComputeLinearIndex(column_index, row_index))];
}

void OccupancyGrid::MarkDisk(double x, double y, double radius, Cell cell) {
  int cx = 0;
  int cy = 0;
  if (!ConvertWorldToCell(x, y, &cx, &cy) || radius <= 0.0) {
    return;
  }
  const int rad = static_cast<int>(std::ceil(radius / resolution_));
  const double r2 = radius * radius;
  for (int dy = -rad; dy <= rad; ++dy) {
    for (int dx = -rad; dx <= rad; ++dx) {
      if ((dx * dx + dy * dy) * resolution_ * resolution_ > r2) {
        continue;
      }
      const int column_index = cx + dx;
      const int row_index = cy + dy;
      if (column_index < 0 || row_index < 0 || column_index >= width_ || row_index >= height_) {
        continue;
      }
      auto& dst = cells_[static_cast<size_t>(ComputeLinearIndex(column_index, row_index))];
      if (cell == Cell::kOccupied && dst != static_cast<uint8_t>(Cell::kOccupied)) {
        dst = static_cast<uint8_t>(Cell::kOccupied);
      }
    }
  }
}

void OccupancyGrid::FreeDisk(double x, double y, double radius) {
  int cx = 0;
  int cy = 0;
  if (!ConvertWorldToCell(x, y, &cx, &cy) || radius <= 0.0) {
    return;
  }
  const int rad = static_cast<int>(std::ceil(radius / resolution_));
  const double r2 = radius * radius;
  for (int dy = -rad; dy <= rad; ++dy) {
    for (int dx = -rad; dx <= rad; ++dx) {
      if ((dx * dx + dy * dy) * resolution_ * resolution_ > r2) {
        continue;
      }
      const int column_index = cx + dx;
      const int row_index = cy + dy;
      if (column_index < 0 || row_index < 0 || column_index >= width_ || row_index >= height_) {
        continue;
      }
      cells_[static_cast<size_t>(ComputeLinearIndex(column_index, row_index))] = static_cast<uint8_t>(Cell::kFree);
    }
  }
}

void OccupancyGrid::BuildHeat(double static_alpha, double static_rmax, double static_hmax,
                              const std::vector<DynObstacle>& obstacles, double now,
                              double prediction_horizon, double heat_alpha0, double heat_alpha1,
                              double tube_radius, double obst_max_vel, double focus_x,
                              double focus_y, double focus_radius) {
  std::fill(heat_.begin(), heat_.end(), 0.0f);
  if (IsEmpty()) {
    return;
  }
  auto add_blob = [&](double world_x, double world_y, double radius, double alpha) {
    if (radius <= 1e-3 || alpha <= 0.0) {
      return;
    }
    int cx = 0;
    int cy = 0;
    if (!ConvertWorldToCell(world_x, world_y, &cx, &cy)) {
      return;
    }
    const int rad = static_cast<int>(std::ceil(radius / resolution_));
    for (int dy = -rad; dy <= rad; ++dy) {
      for (int dx = -rad; dx <= rad; ++dx) {
        const int column_index = cx + dx;
        const int row_index = cy + dy;
        if (column_index < 0 || row_index < 0 || column_index >= width_ || row_index >= height_) {
          continue;
        }
        const double d = std::hypot(dx * resolution_, dy * resolution_);
        if (d >= radius) {
          continue;
        }
        const float h = static_cast<float>(alpha * std::pow(1.0 - d / radius, 2.0));
        auto& cell = heat_[static_cast<size_t>(ComputeLinearIndex(column_index, row_index))];
        cell = std::min(static_cast<float>(static_hmax), std::max(cell, h));
      }
    }
  };

  if (static_alpha > 0.0 && static_rmax > 0.0) {
    int ix_lo = 0;
    int ix_hi = width_ - 1;
    int iy_lo = 0;
    int iy_hi = height_ - 1;
    int fx = 0;
    int fy = 0;
    if (focus_radius > 0.0 && ConvertWorldToCell(focus_x, focus_y, &fx, &fy)) {
      const int rad =
          static_cast<int>(std::ceil((focus_radius + static_rmax) / std::max(resolution_, 1e-3)));
      ix_lo = std::max(0, fx - rad);
      ix_hi = std::min(width_ - 1, fx + rad);
      iy_lo = std::max(0, fy - rad);
      iy_hi = std::min(height_ - 1, fy + rad);
    }
    for (int row_index = iy_lo; row_index <= iy_hi; ++row_index) {
      for (int column_index = ix_lo; column_index <= ix_hi; ++column_index) {
        if (GetCell(column_index, row_index) != Cell::kOccupied) {
          continue;
        }
        const bool boundary = GetCell(column_index - 1, row_index) != Cell::kOccupied ||
                              GetCell(column_index + 1, row_index) != Cell::kOccupied ||
                              GetCell(column_index, row_index - 1) != Cell::kOccupied ||
                              GetCell(column_index, row_index + 1) != Cell::kOccupied;
        if (!boundary) {
          continue;
        }
        double world_x = 0.0;
        double world_y = 0.0;
        ConvertCellToWorld(column_index, row_index, &world_x, &world_y);
        if (focus_radius > 0.0 && std::hypot(world_x - focus_x, world_y - focus_y) > focus_radius) {
          continue;
        }
        add_blob(world_x, world_y, static_rmax, static_alpha);
      }
    }
  }

  const double horizon = std::max(prediction_horizon, 1e-3);
  const int samples =
      std::max(5, std::min(10, static_cast<int>(std::ceil(horizon / 0.5)) + 1));
  const double tau = std::max(1e-3, 0.5 * horizon);
  for (const auto& obs : obstacles) {
    const double age = std::max(0.0, now - obs.last_seen());
    const double px = obs.motion().pose().position().x() + obs.motion().velocity().linear().x() * age;
    const double py = obs.motion().pose().position().y() + obs.motion().velocity().linear().y() * age;
    const double half_extent_x = std::max(obs.half_extent().x(), 0.05);
    const double half_extent_y = std::max(obs.half_extent().y(), 0.05);
    const double base_radius = std::max(half_extent_x, half_extent_y) + std::max(0.0, tube_radius);
    const double reach = base_radius + std::max(0.0, obst_max_vel) * horizon;
    struct Sample {
      double x;
      double y;
      double weight;
      double radius;
    };
    std::vector<Sample> predicted;
    predicted.reserve(static_cast<size_t>(samples));
    double window = std::max(reach, tube_radius);
    for (int j = 0; j < samples; ++j) {
      const double u = samples == 1 ? 0.0 : static_cast<double>(j) / (samples - 1);
      const double t = u * horizon;
      Sample sample;
      sample.x = px + obs.motion().velocity().linear().x() * t;
      sample.y = py + obs.motion().velocity().linear().y() * t;
      sample.weight = std::exp(-t / tau);
      sample.radius = base_radius + std::max(0.0, obst_max_vel) * t;
      predicted.push_back(sample);
      window = std::max(window, std::hypot(sample.x - px, sample.y - py) + sample.radius);
    }
    int cx = 0;
    int cy = 0;
    if (!ConvertWorldToCell(px, py, &cx, &cy)) {
      continue;
    }
    const int rad = static_cast<int>(std::ceil((window + std::max(half_extent_x, half_extent_y)) / resolution_));
    for (int dy = -rad; dy <= rad; ++dy) {
      for (int dx = -rad; dx <= rad; ++dx) {
        const int column_index = cx + dx;
        const int row_index = cy + dy;
        if (column_index < 0 || row_index < 0 || column_index >= width_ || row_index >= height_ || GetCell(column_index, row_index) == Cell::kOccupied) {
          continue;
        }
        double world_x = 0.0;
        double world_y = 0.0;
        ConvertCellToWorld(column_index, row_index, &world_x, &world_y);
        double base = 0.0;
        if (reach > 1e-6) {
          const double dist = std::hypot(world_x - px, world_y - py);
          if (dist <= reach) {
            const double u = dist / reach;
            base = heat_alpha0 * (1.0 - u) * (1.0 - u);
          }
        }
        double tube = 0.0;
        for (const auto& sample : predicted) {
          if (sample.radius <= 1e-6) {
            continue;
          }
          const double dist = std::hypot(world_x - sample.x, world_y - sample.y);
          if (dist > sample.radius) {
            continue;
          }
          const double u = dist / sample.radius;
          tube = std::max(tube, sample.weight * (1.0 - u) * (1.0 - u));
        }
        const double heat = base + heat_alpha1 * tube;
        if (heat <= 0.0) {
          continue;
        }
        auto& cell = heat_[static_cast<size_t>(ComputeLinearIndex(column_index, row_index))];
        cell = std::min(static_cast<float>(static_hmax),
                        std::max(cell, static_cast<float>(heat)));
      }
    }
  }
}

void OccupancyGrid::CollectObstaclePoints(double first_corner_x, double first_corner_y,
                                           double second_corner_x, double second_corner_y,
                                           bool include_unknown,
                                  std::vector<Eigen::Vector2d>* points) const {
  points->clear();
  if (IsEmpty()) {
    return;
  }
  const double margin = 1.0;
  const double query_minimum_x = std::min(first_corner_x, second_corner_x) - margin;
  const double query_maximum_x = std::max(first_corner_x, second_corner_x) + margin;
  const double query_minimum_y = std::min(first_corner_y, second_corner_y) - margin;
  const double query_maximum_y = std::max(first_corner_y, second_corner_y) + margin;
  int minimum_column = 0;
  int minimum_row = 0;
  int maximum_column = 0;
  int maximum_row = 0;
  if (!ConvertWorldToCell(query_minimum_x, query_minimum_y, &minimum_column, &minimum_row)) {
    minimum_column = 0;
    minimum_row = 0;
  }
  if (!ConvertWorldToCell(query_maximum_x, query_maximum_y, &maximum_column, &maximum_row)) {
    maximum_column = width_ - 1;
    maximum_row = height_ - 1;
  }
  if (minimum_column > maximum_column) {
    std::swap(minimum_column, maximum_column);
  }
  if (minimum_row > maximum_row) {
    std::swap(minimum_row, maximum_row);
  }
  for (int row_index = minimum_row; row_index <= maximum_row; row_index += 2) {
    for (int column_index = minimum_column; column_index <= maximum_column; column_index += 2) {
      const Cell cell = GetCell(column_index, row_index);
      if (cell == Cell::kOccupied || (include_unknown && cell == Cell::kUnknown)) {
        double world_x = 0.0;
        double world_y = 0.0;
        ConvertCellToWorld(column_index, row_index, &world_x, &world_y);
        points->emplace_back(world_x, world_y);
      }
    }
  }
}

double OccupancyGrid::ComputeNearestDistance(double x, double y, double search_radius, Cell kind) const {
  int cx = 0;
  int cy = 0;
  if (!ConvertWorldToCell(x, y, &cx, &cy)) {
    return search_radius;
  }
  const int rad = static_cast<int>(std::ceil(search_radius / std::max(resolution_, 1e-3)));
  double best = search_radius;
  for (int dy = -rad; dy <= rad; ++dy) {
    for (int dx = -rad; dx <= rad; ++dx) {
      if (GetCell(cx + dx, cy + dy) != kind) {
        continue;
      }
      best = std::min(best, std::hypot(dx * resolution_, dy * resolution_));
    }
  }
  return best;
}

double OccupancyGrid::ComputeNearestOccupiedDistance(double x, double y, double search_radius) const {
  return ComputeNearestDistance(x, y, search_radius, Cell::kOccupied);
}

}  // namespace sando_controller
}  // namespace controller
}  // namespace control
}  // namespace autonomy
