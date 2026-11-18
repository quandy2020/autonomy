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
 * @file boustrophedon_decomp.cpp
 * @brief Fields2Cover boustrophedon decomp implementation.
 *
 * Ported from Fields2Cover (BSD-3, Wageningen University).
 */



#include "autonomy/planning/planner/fields2cover/decomposition/boustrophedon_decomp.hpp"

namespace autonomy::planning::planner::fields2cover::decomp
{

bool BoustrophedonDecomp::IsLinePartOfLinearRing(
  const F2CLinearRing & ring,
  const F2CLineString & line) const
{
  if (ring.size() == 0 || line.size() == 0) {return false;}

  // Iterate over the edges (segments) of the ring
  for (int i = 0; i < ring.size() - 1; ++i) { // Exclude the last point to avoid looping twice
    auto RingPointStart = ring.GetGeometry(i);
    auto RingPointEnd = ring.GetGeometry(i + 1);

    // Check if the line matches the current edge of the ring
    if ((line.StartPoint() == RingPointStart && line.EndPoint() == RingPointEnd) ||
      (line.StartPoint() == RingPointEnd && line.EndPoint() == RingPointStart))
    {
      // The line matches one of the ring edges
      return true;
    }
  }

  // If the line doesn't match any edge of the ring, return false
  return false;
}

F2CMultiLineString BoustrophedonDecomp::GenSplitLines(
  const F2CCells & cells, const obj::DecompObjective & obj)
{
  F2CMultiLineString split_lines;
  for (auto && cell : cells) {
    for (auto && ring : cell) {
      for (auto && p : ring) {
        auto line1 = cell.CreateLineUntilBorder(p, split_angle);
        while (IsLinePartOfLinearRing(ring, line1)) {
          line1 = cell.CreateLineUntilBorder(line1.EndPoint(), split_angle);
        }
        auto line2 = cell.CreateLineUntilBorder(p, split_angle + M_PI);
        while (IsLinePartOfLinearRing(ring, line2)) {
          line2 = cell.CreateLineUntilBorder(line2.EndPoint(), split_angle + M_PI);
        }

        if (line1.Length() > 1e-5 && line2.Length() > 1e-5) {
          split_lines.AddGeometry(line1);
          split_lines.AddGeometry(line2);
        }
      }
    }
  }
  return split_lines;
}

}  // namespace autonomy::planning::planner::fields2cover::decomp
