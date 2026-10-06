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
 * @file safe_corridor.cpp
 * @brief Ellipsoid decomposition into time-layered half-plane polytopes.
 *
 * Declarations and the algorithm contract live in the matching header.
 * This file holds the definitions.
 */

#include "autonomy/control/controller/sando_controller/safe_corridor.hpp"

#include <algorithm>
#include <cmath>
#include <string>
#include <utility>

#include "autonomy/map/utils/decompose/line_segment.hpp"

namespace autonomy {
namespace control {
namespace controller {
namespace sando_controller {

bool SafeCorridor::Build(const OccupancyGrid& grid, const proto::SandoControllerOptions& options,
                                 const std::vector<DynObstacle>& obstacles,
                                 const std::vector<Eigen::Vector2d>& spatial, int time_layers,
                                 double layer_duration,
                   std::vector<std::vector<std::vector<HalfPlane>>>* layers) const {
  layers->clear();
  if (spatial.size() < 2 || time_layers < 1 || layer_duration <= 0.0) {
    return false;
  }
  std::vector<Eigen::Vector2d> cloud;
  grid.CollectObstaclePoints(spatial.front().x(), spatial.front().y(), spatial.back().x(), spatial.back().y(),
                      options.inflate_unknown(), &cloud);
  const map::utils::Vec2f bbox(options.sfc_bbox(), options.sfc_bbox());
  const std::string assumption = options.environment_assumption();
  const int polytopes = static_cast<int>(spatial.size()) - 1;

  auto obstacle_cloud = [&](double t_end) {
    map::utils::vec_Vec2f obs;
    obs.reserve(cloud.size() + obstacles.size() * 8);
    for (const auto& point : cloud) {
      obs.emplace_back(point.x(), point.y());
    }
    if (options.inflate_unknown() && t_end > 0.0) {
      const double inflate_r = std::max(0.0, options.obst_max_vel()) * t_end;
      if (inflate_r > grid.GetResolution()) {
        const double margin = 1.0 + inflate_r;
        const double minx = std::min(spatial.front().x(), spatial.back().x()) - margin;
        const double maxx = std::max(spatial.front().x(), spatial.back().x()) + margin;
        const double miny = std::min(spatial.front().y(), spatial.back().y()) - margin;
        const double maxy = std::max(spatial.front().y(), spatial.back().y()) + margin;
        int minimum_column = 0;
        int minimum_row = 0;
        int maximum_column = 0;
        int maximum_row = 0;
        if (!grid.ConvertWorldToCell(minx, miny, &minimum_column, &minimum_row)) {
          minimum_column = 0;
          minimum_row = 0;
        }
        if (!grid.ConvertWorldToCell(maxx, maxy, &maximum_column, &maximum_row)) {
          maximum_column = grid.GetWidth() - 1;
          maximum_row = grid.GetHeight() - 1;
        }
        if (minimum_column > maximum_column) {
          std::swap(minimum_column, maximum_column);
        }
        if (minimum_row > maximum_row) {
          std::swap(minimum_row, maximum_row);
        }
        for (int row_index = minimum_row; row_index <= maximum_row; row_index += 2) {
          for (int column_index = minimum_column; column_index <= maximum_column; column_index += 2) {
            if (grid.GetCell(column_index, row_index) != Cell::kUnknown) {
              continue;
            }
            const bool boundary = grid.GetCell(column_index - 1, row_index) != Cell::kUnknown ||
                                  grid.GetCell(column_index + 1, row_index) != Cell::kUnknown ||
                                  grid.GetCell(column_index, row_index - 1) != Cell::kUnknown ||
                                  grid.GetCell(column_index, row_index + 1) != Cell::kUnknown;
            if (!boundary) {
              continue;
            }
            double world_x = 0.0;
            double world_y = 0.0;
            grid.ConvertCellToWorld(column_index, row_index, &world_x, &world_y);
            const double step = std::max(grid.GetResolution(), inflate_r / 4.0);
            const int samples = std::max(1, static_cast<int>(std::ceil(inflate_r / step)));
            for (int sample_row = -samples; sample_row <= samples; ++sample_row) {
              for (int sample_column = -samples; sample_column <= samples; ++sample_column) {
                obs.emplace_back(world_x + sample_column * step, world_y + sample_row * step);
              }
            }
          }
        }
      }
    }
    if (assumption == "static") {
      return obs;
    }
    for (const auto& dyn : obstacles) {
      const double speed = std::hypot(dyn.motion().velocity().linear().x(), dyn.motion().velocity().linear().y());
      if (assumption != "dynamic_worst_case" && speed < options.velocity_threshold()) {
        continue;
      }
      const double cx = dyn.motion().pose().position().x();
      const double cy = dyn.motion().pose().position().y();
      const double reachable_radius = options.obst_max_vel() * t_end + options.comm_delay() * options.obst_max_vel();
      const double half_extent_x = std::max(dyn.half_extent().x(), 0.5 * dyn.radius()) + reachable_radius;
      const double half_extent_y = std::max(dyn.half_extent().y(), 0.5 * dyn.radius()) + reachable_radius;
      const double res = std::max(grid.GetResolution(), 0.05);
      const int mx = std::min(8, static_cast<int>(std::ceil(half_extent_x / res)));
      const int my = std::min(8, static_cast<int>(std::ceil(half_extent_y / res)));
      for (int column_index = -mx; column_index <= mx; ++column_index) {
        const double dx = column_index * res;
        if (std::abs(dx) > half_extent_x) {
          continue;
        }
        for (int row_index = -my; row_index <= my; ++row_index) {
          const double dy = row_index * res;
          if (std::abs(dy) > half_extent_y) {
            continue;
          }
          obs.emplace_back(cx + dx, cy + dy);
        }
      }
    }
    return obs;
  };
  auto decompose = [&](const map::utils::vec_Vec2f& obs) {
    std::vector<std::vector<HalfPlane>> layer(static_cast<size_t>(polytopes));
    for (int p = 0; p < polytopes; ++p) {
      map::utils::LineSegment<2> segment(
          map::utils::Vec2f(spatial[static_cast<size_t>(p)].x(), spatial[static_cast<size_t>(p)].y()),
          map::utils::Vec2f(spatial[static_cast<size_t>(p + 1)].x(),
                            spatial[static_cast<size_t>(p + 1)].y()));
      segment.set_local_bbox(bbox);
      segment.set_obs(obs);
      segment.dilate(0.0);
      for (const auto& plane : segment.get_polyhedron().hyperplanes()) {
        HalfPlane half;
        half.mutable_normal()->set_x(plane.n_.x());
        half.mutable_normal()->set_y(plane.n_.y());
        half.set_offset(plane.n_.dot(plane.p_) - std::max(0.0, options.robot_radius()) * plane.n_.norm());
        layer[static_cast<size_t>(p)].push_back(half);
      }
    }
    return layer;
  };
  auto usable = [](const std::vector<std::vector<HalfPlane>>& layer) {
    for (const auto& planes : layer) {
      if (!planes.empty()) {
        return true;
      }
    }
    return false;
  };

  if (assumption == "static") {
    const auto layer = decompose(obstacle_cloud(0.0));
    if (!usable(layer)) {
      return false;
    }
    layers->assign(static_cast<size_t>(time_layers), layer);
    return true;
  }
  layers->reserve(static_cast<size_t>(time_layers));
  for (int n = 0; n < time_layers; ++n) {
    const double t_end = static_cast<double>(n + 1) * layer_duration;
    auto layer = decompose(obstacle_cloud(t_end));
    if (!usable(layer)) {
      return false;
    }
    layers->push_back(std::move(layer));
  }
  return static_cast<int>(layers->size()) == time_layers;
}

}  // namespace sando_controller
}  // namespace controller
}  // namespace control
}  // namespace autonomy
