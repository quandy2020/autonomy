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

  // Overlay individual footprints in a local frame. A multi-line buffer can
  // discard components when rotated strips share almost coincident edges.
  const F2CPoint origin = swaths[0].StartPoint();
  const double angle = swaths[0].GetInAngle();
  const F2CPoint zero(0.0, 0.0);
  auto local_poly = zero.RotateFromPoint(-angle, poly - origin);
  F2CCells footprints;
  for (const auto& swath : swaths) {
    auto line = zero.RotateFromPoint(-angle, swath.GetPath() - origin);
    auto strip = F2CCells::buffer(line, swath.GetWidth() / 2.0);
    footprints = footprints.UnionOp(strip);
  }
  const double covered = local_poly.GetCellsInside(footprints).Area();
  return covered / (local_poly.Area() + 1e-7);
}

bool FieldCoverage::IsMinimizing() const {
  return false;
}

}  // namespace autonomy::planning::planner::fields2cover::obj

