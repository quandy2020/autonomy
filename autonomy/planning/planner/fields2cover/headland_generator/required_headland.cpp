/*
 * Copyright 2026 Autodriver contributors duyongquan (quandy2020@126.com)
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

/**
 * @file required_headland.cpp
 * @brief Fields2Cover required headland implementation.
 *
 * Ported from Fields2Cover (BSD-3, Wageningen University).
 */



#include <vector>
#include "autonomy/planning/planner/fields2cover/headland_generator/required_headland.hpp"

namespace autonomy::planning::planner::fields2cover::hg {

F2CCells ReqHL::GenerateHeadlands(
    const F2CCell& field, const F2CRobot& robot, double track_ang) {
  double max_req_dist = this->MaxHLWidthRequired(robot);
  F2CCells inner_cells {F2CCells::buffer(field, -max_req_dist)};

  F2CCells new_cells;
  for (auto&& inner_cell : inner_cells) {
    F2CCell new_cell;
    for (size_t i = 0; i < inner_cell.size(); ++i) {
      auto ring = inner_cell[i];
      auto req_dist = this->RequiredHeadlandDist(robot, track_ang, ring);

      // Give each border back what it does not need to turn on. Obstacles
      // are the other way around: their borders grow into the mainland.
      for (auto&& d : req_dist) {
        d = (max_req_dist - d) * (i == 0 ? 1.0 : -1.0);
      }
      new_cell.AddGeometry(ring.BufferOutwards(req_dist));
      // Close the hairline spurs offsetting each segment on its own leaves.
      new_cell = F2CCell::buffer(F2CCell::buffer(new_cell, 1e-5), -1e-5);
    }
    new_cells.AddGeometry(new_cell);
    new_cells = F2CCells::buffer(F2CCells::buffer(new_cells, 1e-5), -1e-5);
  }
  return new_cells;
}

F2CCells ReqHL::GenerateHeadlands(
    const F2CCells& field, const F2CRobot& robot,
    const std::vector<double>& track_angs) {
  if (field.size() != track_angs.size()) {
    return GenerateHeadlands(field, MaxHLWidthRequired(robot));
  }
  F2CCells cells;
  for (size_t i = 0; i < field.size(); ++i) {
    auto c_hl = GenerateHeadlands(field[i], robot, track_angs[i]);
    for (auto&& c : c_hl) {
      cells.AddGeometry(c);
    }
  }
  return cells;
}

F2CCells ReqHL::GenerateHeadlands(
    const F2CCells& field, double dist_headland) {
  return field.buffer(-dist_headland);
}

F2CCells ReqHL::GenerateHeadlandArea(
    const F2CCells& field, double swath_width, int n_swaths) {
  return field.buffer(-swath_width * n_swaths);
}

std::vector<F2CCells> ReqHL::GenerateHeadlandSwaths(
    const F2CCells& field, double swath_width, int n_swaths, bool dir_out2in) {
  std::vector<F2CCells> hl;
  if (dir_out2in) {
    for (int i = 0; i < n_swaths; ++i) {
      hl.emplace_back(field.buffer(-swath_width * (i + 0.5)));
    }
  } else {
    hl.emplace_back(field.buffer(-swath_width * (n_swaths - 0.5)));
    for (int i = 1; i < n_swaths; ++i) {
      hl.emplace_back(hl.back().buffer(swath_width));
    }
  }
  return hl;
}

std::vector<double> ReqHL::RequiredHeadlandDist(const F2CRobot& robot,
    double track_ang, const F2CLinearRing& ring) const {
  std::vector<double> v_dist;
  for (size_t i = 1; i < ring.size(); ++i) {
    v_dist.emplace_back(this->RequiredHeadlandDist(
          robot, track_ang, (ring[i] - ring[i-1]).GetAngleFromPoint()));
  }
  return v_dist;
}

double ReqHL::RequiredHeadlandDist(const F2CRobot& robot,
    double track_ang, double border_ang) const {
  return (1.0 + fabs(sin(track_ang - border_ang))) *
    robot.GetMinTurningRadius() + 0.5 * robot.GetWidth();
}

}  // namespace autonomy::planning::planner::fields2cover::hg
