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
 * @file obstacle_tracker.cpp
 * @brief Flood-fill clustering, constant-acceleration filtering, and external-track association.
 *
 * Declarations and the algorithm contract live in the matching header.
 * This file holds the definitions.
 */

#include "autonomy/control/controller/sando_controller/obstacle_tracker.hpp"

#include <algorithm>
#include <cmath>
#include <utility>

namespace autonomy {
namespace control {
namespace controller {
namespace sando_controller {

void ObstacleTracker::AddObstacle(double x, double y, double velocity_x, double velocity_y, double radius,
                          double robot_radius) {
  for (auto& obs : obstacles_) {
    if (obs.external() && std::hypot(obs.motion().pose().position().x() - x, obs.motion().pose().position().y() - y) < 0.4) {
      obs.mutable_motion()->mutable_pose()->mutable_position()->set_x(x);
      obs.mutable_motion()->mutable_pose()->mutable_position()->set_y(y);
      obs.mutable_motion()->mutable_velocity()->mutable_linear()->set_x(velocity_x);
      obs.mutable_motion()->mutable_velocity()->mutable_linear()->set_y(velocity_y);
      obs.set_radius(std::max(radius, robot_radius));
      obs.mutable_half_extent()->set_x(obs.radius());
      obs.mutable_half_extent()->set_y(obs.radius());
      obs.set_last_seen(0.0);
      return;
    }
  }
  DynObstacle obs;
  obs.set_identifier(next_identifier_++);
  obs.mutable_motion()->mutable_pose()->mutable_position()->set_x(x);
  obs.mutable_motion()->mutable_pose()->mutable_position()->set_y(y);
  obs.mutable_motion()->mutable_velocity()->mutable_linear()->set_x(velocity_x);
  obs.mutable_motion()->mutable_velocity()->mutable_linear()->set_y(velocity_y);
  obs.set_radius(std::max(radius, 0.2));
  obs.mutable_half_extent()->set_x(obs.radius());
  obs.mutable_half_extent()->set_y(obs.radius());
  obs.set_external(true);
  obstacles_.push_back(obs);
}

void ObstacleTracker::TrackObstacles(const OccupancyGrid& grid, const proto::SandoControllerOptions& options,
                            double now, double robot_x, double robot_y) {
  if (grid.IsEmpty()) {
    return;
  }
  for (auto& obs : obstacles_) {
    if (obs.external() && obs.last_seen() <= 0.0) {
      obs.set_last_seen(now);
    }
  }
  const double window = options.horizon() + options.map_buffer();
  std::vector<uint8_t> seen(static_cast<size_t>(grid.GetWidth() * grid.GetHeight()), 0);
  struct Blob {
    double x{0.0};
    double y{0.0};
    double min_x{0.0};
    double min_y{0.0};
    double max_x{0.0};
    double max_y{0.0};
    int count{0};
  };
  std::vector<Blob> blobs;
  int minimum_column = 0;
  int maximum_column = -1;
  int minimum_row = 0;
  int maximum_row = -1;
  int robot_column = 0;
  int robot_row = 0;
  if (grid.ConvertWorldToCell(robot_x, robot_y, &robot_column, &robot_row)) {
    const int rad = std::max(1, static_cast<int>(std::ceil(window / grid.GetResolution())));
    minimum_column = std::max(0, robot_column - rad);
    maximum_column = std::min(grid.GetWidth() - 1, robot_column + rad);
    minimum_row = std::max(0, robot_row - rad);
    maximum_row = std::min(grid.GetHeight() - 1, robot_row + rad);
  }
  for (int row_index = minimum_row; row_index <= maximum_row; ++row_index) {
    for (int column_index = minimum_column; column_index <= maximum_column; ++column_index) {
      const int index = column_index + row_index * grid.GetWidth();
      if (seen[static_cast<size_t>(index)] || grid.GetCell(column_index, row_index) != Cell::kOccupied) {
        continue;
      }
      double seed_x = 0.0;
      double seed_y = 0.0;
      grid.ConvertCellToWorld(column_index, row_index, &seed_x, &seed_y);
      if (std::hypot(seed_x - robot_x, seed_y - robot_y) > window) {
        continue;
      }
      std::vector<std::pair<int, int>> stack;
      stack.emplace_back(column_index, row_index);
      seen[static_cast<size_t>(index)] = 1;
      double centroid_sum_x = 0.0;
      double centroid_sum_y = 0.0;
      double min_x = 1.0e30;
      double min_y = 1.0e30;
      double max_x = -1.0e30;
      double max_y = -1.0e30;
      int count = 0;
      while (!stack.empty()) {
        const auto [cx, cy] = stack.back();
        stack.pop_back();
        double world_x = 0.0;
        double world_y = 0.0;
        grid.ConvertCellToWorld(cx, cy, &world_x, &world_y);
        centroid_sum_x += world_x;
        centroid_sum_y += world_y;
        min_x = std::min(min_x, world_x);
        min_y = std::min(min_y, world_y);
        max_x = std::max(max_x, world_x);
        max_y = std::max(max_y, world_y);
        ++count;
        const int nbs[4][2] = {{1, 0}, {-1, 0}, {0, 1}, {0, -1}};
        for (const auto& nb : nbs) {
          const int nx = cx + nb[0];
          const int ny = cy + nb[1];
          if (nx < 0 || ny < 0 || nx >= grid.GetWidth() || ny >= grid.GetHeight()) {
            continue;
          }
          const int ni = nx + ny * grid.GetWidth();
          if (seen[static_cast<size_t>(ni)] || grid.GetCell(nx, ny) != Cell::kOccupied) {
            continue;
          }
          seen[static_cast<size_t>(ni)] = 1;
          stack.emplace_back(nx, ny);
        }
      }
      if (count >= options.min_cluster_cells() && count <= options.max_cluster_cells()) {
        blobs.push_back(Blob{centroid_sum_x / count, centroid_sum_y / count, min_x, min_y, max_x, max_y, count});
      }
    }
  }

  const int known = static_cast<int>(obstacles_.size());
  std::vector<uint8_t> matched(static_cast<size_t>(known), 0);
  for (const auto& blob : blobs) {
    int best = -1;
    double best_d = 1.2;
    for (int i = 0; i < known; ++i) {
      if (matched[static_cast<size_t>(i)] || obstacles_[static_cast<size_t>(i)].external()) {
        continue;
      }
      const double d = std::hypot(obstacles_[static_cast<size_t>(i)].motion().pose().position().x() - blob.x,
                                   obstacles_[static_cast<size_t>(i)].motion().pose().position().y() - blob.y);
      if (d < best_d) {
        best_d = d;
        best = i;
      }
    }
    if (best >= 0) {
      auto& obs = obstacles_[static_cast<size_t>(best)];
      const double dt = std::max(1e-3, now - obs.last_seen());
      const double measured_velocity_x = (blob.x - obs.motion().pose().position().x()) / dt;
      const double measured_velocity_y = (blob.y - obs.motion().pose().position().y()) / dt;
      const double measured_acceleration_x = (measured_velocity_x - obs.motion().velocity().linear().x()) / dt;
      const double measured_acceleration_y = (measured_velocity_y - obs.motion().velocity().linear().y()) / dt;
      const double alpha = options.kf_alpha();
      obs.mutable_motion()->mutable_acceleration()->mutable_linear()->set_x(alpha * obs.motion().acceleration().linear().x() + (1.0 - alpha) * measured_acceleration_x);
      obs.mutable_motion()->mutable_acceleration()->mutable_linear()->set_y(alpha * obs.motion().acceleration().linear().y() + (1.0 - alpha) * measured_acceleration_y);
      obs.mutable_motion()->mutable_velocity()->mutable_linear()->set_x(alpha * obs.motion().velocity().linear().x() + (1.0 - alpha) * measured_velocity_x);
      obs.mutable_motion()->mutable_velocity()->mutable_linear()->set_y(alpha * obs.motion().velocity().linear().y() + (1.0 - alpha) * measured_velocity_y);
      const double speed = std::hypot(obs.motion().velocity().linear().x(), obs.motion().velocity().linear().y());
      const double accel = std::hypot(obs.motion().acceleration().linear().x(), obs.motion().acceleration().linear().y());
      if (speed > options.obst_max_vel()) {
        obs.mutable_motion()->mutable_velocity()->mutable_linear()->set_x(
            obs.motion().velocity().linear().x() * options.obst_max_vel() / speed);
        obs.mutable_motion()->mutable_velocity()->mutable_linear()->set_y(
            obs.motion().velocity().linear().y() * options.obst_max_vel() / speed);
      }
      if (speed < options.velocity_threshold() && accel < options.accel_threshold()) {
        obs.mutable_motion()->mutable_velocity()->mutable_linear()->set_x(0.0);
        obs.mutable_motion()->mutable_velocity()->mutable_linear()->set_y(0.0);
        obs.mutable_motion()->mutable_acceleration()->mutable_linear()->set_x(0.0);
        obs.mutable_motion()->mutable_acceleration()->mutable_linear()->set_y(0.0);
      }
      obs.mutable_motion()->mutable_pose()->mutable_position()->set_x(blob.x);
      obs.mutable_motion()->mutable_pose()->mutable_position()->set_y(blob.y);
      obs.set_last_seen(now);
      const double half_extent = std::max(0.05, 0.5 * std::max(blob.max_x - blob.min_x, blob.max_y - blob.min_y));
      obs.mutable_half_extent()->set_x(half_extent);
      obs.mutable_half_extent()->set_y(half_extent);
      obs.set_radius(std::max(options.robot_radius(), half_extent * std::sqrt(2.0)));
      matched[static_cast<size_t>(best)] = 1;
    } else if (obstacles_.size() < 16) {
      DynObstacle obs;
      obs.set_identifier(next_identifier_++);
      obs.mutable_motion()->mutable_pose()->mutable_position()->set_x(blob.x);
      obs.mutable_motion()->mutable_pose()->mutable_position()->set_y(blob.y);
      obs.set_last_seen(now);
      const double half_extent = std::max(0.05, 0.5 * std::max(blob.max_x - blob.min_x, blob.max_y - blob.min_y));
      obs.mutable_half_extent()->set_x(half_extent);
      obs.mutable_half_extent()->set_y(half_extent);
      obs.set_radius(std::max(options.robot_radius(), half_extent * std::sqrt(2.0)));
      obstacles_.push_back(obs);
    }
  }
  obstacles_.erase(std::remove_if(obstacles_.begin(), obstacles_.end(),
                                  [&](const DynObstacle& obs) {
                                    return now - obs.last_seen() > options.traj_lifetime();
                                  }),
                   obstacles_.end());
}

}  // namespace sando_controller
}  // namespace controller
}  // namespace control
}  // namespace autonomy
