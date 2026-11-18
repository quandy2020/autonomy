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
 * @file field_coverage.cpp
 * @brief Fields2Cover field coverage implementation.
 *
 * Ported from Fields2Cover (BSD-3, Wageningen University).
 */



#include "autonomy/planning/planner/fields2cover/objectives/sg_obj/field_coverage.hpp"

namespace autonomy::planning::planner::fields2cover::obj {

double FieldCoverage::ComputeCost(
    const F2CCell& poly, const F2CSwaths& swaths) {
  return ComputeCost(F2CCells(poly), swaths);
}

double FieldCoverage::ComputeCost(
    const F2CCells& poly, const F2CSwaths& swaths) {
  if (swaths.size() == 0) {
    return 0.0;
  }

  F2CMultiLineString lines;
  for (const auto& s : swaths) {
    lines.AddGeometry(s.GetPath());
  }

  double area_covered {poly.GetCellsInside(
      F2CCells::buffer(lines, swaths[0].GetWidth() / 2.0)).Area()};
  return area_covered / (poly.Area() + 1e-7);
}

bool FieldCoverage::IsMinimizing() const {
  return false;
}

}  // namespace autonomy::planning::planner::fields2cover::obj

