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
 * @file corridor_headland.cpp
 * @brief Fields2Cover corridor headland implementation.
 *
 * Ported from Fields2Cover (BSD-3, Wageningen University).
 */



#include <algorithm>
#include <cmath>
#include <vector>
#include "autonomy/planning/planner/fields2cover/headland_generator/corridor_headland.hpp"

namespace autonomy::planning::planner::fields2cover::hg {

std::vector<CorridorShare> CorridorHL::CorridorShares(
    const F2CCells& field, CorridorShareMode mode) const {
  // The corridor comes out of the smaller cell, so the larger neighbour keeps
  // its shape: taking half from each side notches both, and a notched cell
  // costs the swath Generator a pass. In SYMMETRIC mode every border splits
  // evenly instead, regardless of size, for comparison against that rule.
  std::vector<CorridorShare> shares;
  for (size_t i = 0; i < field.size(); ++i) {
    const F2CLinearRing ring = field.GetCellBorder(i);
    const double perimeter = ring.Length();
    for (size_t k = 0; k < field.size(); ++k) {
      if (k == i) {
        continue;
      }
      CorridorShare share;
      share.cell_i = i;
      share.cell_k = k;
      share.perimeter_i = perimeter;
      share.perimeter_k = field.GetCellBorder(k).Length();
      // Cells that are the same size share the corridor, each giving half.
      // The comparison has to be loose: perimeters that are equal in theory
      // differ in the last bits, and letting that noise pick a winner leaves
      // some borders with a full corridor and others with none.
      share.same_size = std::abs(perimeter - share.perimeter_k) <=
          same_size_tol_ * perimeter;
      share.share = mode == CorridorShareMode::SYMMETRIC ? 0.5 :
          (share.same_size ? 0.5 :
              (perimeter < share.perimeter_k ? 1.0 : 0.0));
      // Keep the part of each border edge that the neighbour actually touches.
      const F2CCells neighbour = F2CCells::buffer(field.GetGeometry(k), tol_);
      for (size_t e = 0; e + 1 < ring.size(); ++e) {
        F2CMultiLineString edge;
        edge.AddGeometry(
            F2CLineString({ring.GetGeometry(e), ring.GetGeometry(e + 1)}));
        const F2CMultiLineString shared = edge.intersection(neighbour);
        for (size_t j = 0; j < shared.size(); ++j) {
          const F2CLineString part = shared.GetGeometry(j);
          if (part.size() > 1 && part.Length() > min_border_) {
            share.shared_border.AddGeometry(part);
            share.shared_length += part.Length();
          }
        }
      }
      // Cells that only meet at a corner, or not at all, are not neighbours.
      if (share.shared_length > 0.0) {
        shares.emplace_back(share);
      }
    }
  }
  return shares;
}

void CorridorHL::SetShareMode(CorridorShareMode mode) {
  share_mode_ = mode;
}

CorridorShareMode CorridorHL::GetShareMode() const {
  return share_mode_;
}

F2CCells CorridorHL::GenerateHeadlands(
    const F2CCells& field, double dist_headland) {
  const std::vector<CorridorShare> shares = CorridorShares(field, share_mode_);
  F2CCells carved;
  for (size_t i = 0; i < field.size(); ++i) {
    F2CCells cell {field.GetGeometry(i)};
    for (const CorridorShare& share : shares) {
      if (share.cell_i != i || share.share <= 0.0) {
        continue;  // this cell gives no corridor on that border
      }
      for (size_t j = 0; j < share.shared_border.size(); ++j) {
        cell = cell.difference(F2CCells::buffer(
            share.shared_border.GetGeometry(j), dist_headland * share.share));
      }
    }
    // The difference can leave a zero-width spur behind. Opening the result by
    // a hair drops it without moving any real edge.
    const F2CCells clean = cell.buffer(-spur_).buffer(spur_);
    for (size_t j = 0; j < clean.size(); ++j) {
      carved.AddGeometry(clean.GetGeometry(j));
    }
  }
  return carved;
}

F2CCells CorridorHL::GenerateHeadlandArea(
    const F2CCells& field, double swath_width, int n_swaths) {
  return GenerateHeadlands(field, swath_width * n_swaths);
}

std::vector<F2CCells> CorridorHL::GenerateHeadlandSwaths(
    const F2CCells& field, double swath_width, int n_swaths, bool dir_out2in) {
  std::vector<F2CCells> hl;
  for (int i = 0; i < n_swaths; ++i) {
    const int n = dir_out2in ? (n_swaths - i) : (i + 1);
    hl.emplace_back(GenerateHeadlands(field, swath_width * (n - 0.5)));
  }
  return hl;
}


}  // namespace autonomy::planning::planner::fields2cover::hg
