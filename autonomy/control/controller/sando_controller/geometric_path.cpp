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
 * @file geometric_path.cpp
 * @brief Arc-length resampling and truncation of the path before unknown space.
 *
 * Declarations and the algorithm contract live in the matching header.
 * This file holds the definitions.
 */

#include "autonomy/control/controller/sando_controller/geometric_path.hpp"

#include <algorithm>
#include <cmath>
#include <utility>

namespace autonomy {
namespace control {
namespace controller {
namespace sando_controller {

std::vector<Eigen::Vector2d> GeometricPath::Resample(const std::vector<Eigen::Vector2d>& in, double spacing,
                                                        int max_points) const {
  std::vector<Eigen::Vector2d> out;
  if (in.empty()) {
    return out;
  }
  out.push_back(in.front());
  double acc = 0.0;
  for (size_t i = 1; i < in.size(); ++i) {
    const Eigen::Vector2d delta = in[i] - in[i - 1];
    const double len = delta.norm();
    if (len < 1e-4) {
      continue;
    }
    const Eigen::Vector2d dir = delta / len;
    double traveled = 0.0;
    while (acc + (len - traveled) >= spacing) {
      const double need = spacing - acc;
      traveled += need;
      out.push_back(in[i - 1] + dir * traveled);
      acc = 0.0;
      if (static_cast<int>(out.size()) >= max_points) {
        out.back() = in.back();
        return out;
      }
    }
    acc += len - traveled;
  }
  if ((out.back() - in.back()).norm() > 1e-3) {
    out.push_back(in.back());
  }
  while (out.size() < 2 && !in.empty()) {
    out.push_back(in.back());
  }
  if (static_cast<int>(out.size()) > max_points) {
    std::vector<Eigen::Vector2d> trimmed;
    trimmed.reserve(static_cast<size_t>(max_points));
    for (int i = 0; i < max_points; ++i) {
      const double u = static_cast<double>(i) / std::max(1, max_points - 1);
      const double f = u * static_cast<double>(out.size() - 1);
      const int i0 = static_cast<int>(std::floor(f));
      const int i1 = std::min(i0 + 1, static_cast<int>(out.size()) - 1);
      const double t = f - i0;
      trimmed.push_back((1.0 - t) * out[static_cast<size_t>(i0)] +
                        t * out[static_cast<size_t>(i1)]);
    }
    trimmed.back() = out.back();
    return trimmed;
  }
  return out;
}

void GeometricPath::Truncate(const OccupancyGrid& grid, const proto::SandoControllerOptions& options,
                                   std::vector<Eigen::Vector2d>* path) const {
  if (path->size() < 2) {
    return;
  }
  const double sample = 0.1;
  const double touch = options.robot_radius();
  const double inflate = options.robot_radius() + options.obst_max_vel() * options.prediction_horizon();
  auto unknown_near = [&](const Eigen::Vector2d& p, double radius) {
    return grid.ComputeNearestDistance(p.x(), p.y(), radius + grid.GetResolution(), Cell::kUnknown) < radius;
  };
  std::vector<Eigen::Vector2d> kept;
  kept.push_back(path->front());
  for (size_t i = 0; i + 1 < path->size(); ++i) {
    const Eigen::Vector2d a = (*path)[i];
    const Eigen::Vector2d b = (*path)[i + 1];
    const double len = (b - a).norm();
    if (len < 1e-4) {
      continue;
    }
    const Eigen::Vector2d dir = (b - a) / len;
    bool hit = false;
    for (double s = 0.0; s <= len; s += sample) {
      const Eigen::Vector2d p = a + dir * s;
      if (!unknown_near(p, touch)) {
        continue;
      }
      int seg = static_cast<int>(i);
      double back = s;
      Eigen::Vector2d safe = p;
      Eigen::Vector2d sa = a;
      Eigen::Vector2d sb = b;
      Eigen::Vector2d sdir = dir;
      while (unknown_near(safe, inflate)) {
        back -= sample;
        if (back >= 0.0) {
          safe = sa + sdir * back;
          continue;
        }
        --seg;
        if (seg < 0) {
          safe = path->front();
          break;
        }
        sa = (*path)[static_cast<size_t>(seg)];
        sb = (*path)[static_cast<size_t>(seg + 1)];
        const double seglen = (sb - sa).norm();
        if (seglen < 1e-4) {
          back = 0.0;
          safe = sa;
          continue;
        }
        sdir = (sb - sa) / seglen;
        back = seglen + back;
        if (back < 0.0) {
          back = 0.0;
        }
        safe = sa + sdir * back;
      }
      if ((safe - kept.back()).norm() > 1e-3) {
        kept.push_back(safe);
      }
      hit = true;
      break;
    }
    if (hit) {
      *path = std::move(kept);
      return;
    }
    kept.push_back(b);
  }
  *path = std::move(kept);
}

}  // namespace sando_controller
}  // namespace controller
}  // namespace control
}  // namespace autonomy
