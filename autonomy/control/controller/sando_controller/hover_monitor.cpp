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
 * @file hover_monitor.cpp
 * @brief Repulsion headings that slide the working hover goal off a threat.
 *
 * Declarations and the algorithm contract live in the matching header.
 * This file holds the definitions.
 */

#include "autonomy/control/controller/sando_controller/hover_monitor.hpp"

#include <algorithm>
#include <cmath>

#include "autonomy/control/controller/sando_controller/position_safety.hpp"

namespace autonomy {
namespace control {
namespace controller {
namespace sando_controller {

void HoverMonitor::Evade(const OccupancyGrid& grid, const proto::SandoControllerOptions& options,
                                     const std::vector<DynObstacle>& obstacles, const State& robot,
                                     double hover_x, double hover_y, State* goal) const {
  const bool robot_threat = position_safety_.IsPositionThreatened(grid, options, obstacles, robot.pose().position().x(), robot.pose().position().y(), true) || position_safety_.IsCellOccupied(grid, robot.pose().position().x(), robot.pose().position().y());
  const bool hover_threat = position_safety_.IsPositionThreatened(grid, options, obstacles, hover_x, hover_y, true) || position_safety_.IsCellOccupied(grid, hover_x, hover_y);
  if (robot_threat) {
    double rx = 0.0;
    double ry = 0.0;
    int column_index = 0;
    int row_index = 0;
    if (grid.ConvertWorldToCell(robot.pose().position().x(), robot.pose().position().y(), &column_index, &row_index)) {
      const int rad = static_cast<int>(std::ceil(options.hover_d_trigger() / grid.GetResolution()));
      for (int dy = -rad; dy <= rad; ++dy) {
        for (int dx = -rad; dx <= rad; ++dx) {
          if (grid.GetCell(column_index + dx, row_index + dy) != Cell::kOccupied) {
            continue;
          }
          const double d = std::hypot(dx, dy) * grid.GetResolution();
          if (d < 1e-3 || d > options.hover_d_trigger()) {
            continue;
          }
          rx -= static_cast<double>(dx) / (d * d);
          ry -= static_cast<double>(dy) / (d * d);
        }
      }
    }
    for (const auto& obs : obstacles) {
      const double dx = robot.pose().position().x() - obs.motion().pose().position().x();
      const double dy = robot.pose().position().y() - obs.motion().pose().position().y();
      const double d = std::hypot(dx, dy);
      if (d < 1e-3 || d > options.hover_d_trigger() + obs.radius()) {
        continue;
      }
      rx += dx / (d * d);
      ry += dy / (d * d);
    }
    const double n = std::hypot(rx, ry);
    const double base = n > 1e-3 ? std::atan2(ry, rx) : Yaw(robot);
    constexpr double kRadiansPerDegree = 3.14159265358979323846 / 180.0;
    const double angles[] = {0.0, 30.0, -30.0, 60.0, -60.0, 90.0, -90.0, 180.0};
    for (double deg : angles) {
      const double dir = base + deg * kRadiansPerDegree;
      const double ex = robot.pose().position().x() + options.hover_evasion() * std::cos(dir);
      const double ey = robot.pose().position().y() + options.hover_evasion() * std::sin(dir);
      if (!position_safety_.IsPositionThreatened(grid, options, obstacles, ex, ey, true) && !position_safety_.IsCellOccupied(grid, ex, ey)) {
        goal->mutable_pose()->mutable_position()->set_x(ex);
        goal->mutable_pose()->mutable_position()->set_y(ey);
        break;
      }
    }
  } else if (!hover_threat) {
    goal->mutable_pose()->mutable_position()->set_x(hover_x);
    goal->mutable_pose()->mutable_position()->set_y(hover_y);
  }
}

}  // namespace sando_controller
}  // namespace controller
}  // namespace control
}  // namespace autonomy
