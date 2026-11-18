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
 * @file swath_generator_base.cpp
 * @brief Fields2Cover swath generator base implementation.
 *
 * Ported from Fields2Cover (BSD-3, Wageningen University).
 */



#include <boost/math/constants/constants.hpp>
#include "autonomy/planning/planner/fields2cover/swath_generator/swath_generator_base.hpp"

namespace autonomy::planning::planner::fields2cover::sg {

bool SwathGeneratorBase::GetAllowOverlap() const {
  return this->allow_overlap;
}

void SwathGeneratorBase::SetAllowOverlap(bool value) {
  this->allow_overlap = value;
}

F2CSwaths SwathGeneratorBase::GenerateBestSwaths(
    autonomy::planning::planner::fields2cover::obj::SGObjective& obj, double op_width, const F2CCell& poly) {
  const double ang = ComputeBestAngle(obj, op_width, poly);
  F2CSwaths swaths = GenerateSwaths(ang, op_width, poly);
  if (swaths.size() > 0) {
    return swaths;
  }
  // The objectives score an angle from the cell border alone, so on a cell with
  // a hairline spur the cheapest angle can cover nothing and the cell is skipped
  // without a word. Sweep for an angle that does cover it.
  constexpr int KTries = 8;
  for (int i = 1; i < KTries; ++i) {
    F2CSwaths alt = GenerateSwaths(
        ang + i * boost::math::constants::pi<double>() / KTries,
        op_width, poly);
    if (alt.size() > 0) {
      return alt;
    }
  }
  return swaths;
}

F2CSwathsByCells SwathGeneratorBase::GenerateBestSwaths(
    autonomy::planning::planner::fields2cover::obj::SGObjective& obj, double op_width, const F2CCells& polys) {
  F2CSwathsByCells swaths;
  for (auto&& p : polys) {
    swaths.emplace_back(GenerateBestSwaths(obj, op_width, p));
  }
  return swaths;
}

F2CSwathsByCells SwathGeneratorBase::GenerateSwaths(double angle,
    double op_width, const F2CCells& polys) {
  F2CSwathsByCells swaths;
  for (auto&& p : polys) {
    swaths.emplace_back(GenerateSwaths(angle, op_width, p));
  }
  return swaths;
}


F2CSwaths SwathGeneratorBase::GenerateSwaths(double angle,
    double op_width, const F2CCell& poly) {
  auto rot_poly {F2CPoint(0.0, 0.0).RotateFromPoint(-angle, poly)};

  double field_height {rot_poly.GetHeight()};
  F2CPoint min_point(rot_poly.GetDimMinX(), rot_poly.GetDimMinY());
  auto seed_curve = rot_poly.CreateStraightLongLine(min_point, 0.0);

  double curve_y {-0.5 * op_width};
  F2CMultiLineString paths;
  while (field_height > curve_y + (allow_overlap ? 0.0 : 0.5 * op_width)) {
    curve_y += op_width;
    paths.AddGeometry(F2CPoint(0.0, 0.0).RotateFromPoint(angle,
        seed_curve + F2CPoint(0.0, curve_y)));
  }

  F2CSwaths swaths;
  swaths.Append(paths, poly, op_width, autonomy::planning::planner::fields2cover::types::SwathType::MAINLAND);
  return swaths;
}


double SwathGeneratorBase::ComputeCostOfAngle(autonomy::planning::planner::fields2cover::obj::SGObjective& obj,
    double ang, double op_width, const F2CCell& poly) {
  if (obj.IsFastCompAvailable()) {
    return obj.ComputeCostWithMinimizingSign(ang, op_width, poly);
  }
  return obj.ComputeCostWithMinimizingSign(poly,
      GenerateSwaths(ang, op_width, poly));
}

}  // namespace autonomy::planning::planner::fields2cover::sg

