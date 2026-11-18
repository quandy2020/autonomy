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
#include <cmath>
#include <stdexcept>
#include "autonomy/planning/planner/fields2cover/objectives/sg_obj/field_coverage.hpp"
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
  obj::FieldCoverage coverage;
  double score = coverage.ComputeCost(poly, swaths);
  // Border-only objectives do not see missed narrow arms or clipped end caps.
  // Retain their choice when it covers the field well; otherwise search exact
  // border headings and other orientations for a more complete footprint.
  constexpr double kCoverageTarget = .98;
  if (swaths.size() > 0 && score >= kCoverageTarget) return swaths;
  std::vector<double> angles;
  for (const auto& ring : poly) {
    for (size_t i = 1; i < ring.size(); ++i) {
      if (ring[i].Distance(ring[i - 1]) < 1e-8) continue;
      const double a = (ring[i] - ring[i - 1]).GetAngleFromPoint();
      angles.push_back(a);
      angles.push_back(a + M_PI / 2.0);
    }
  }
  for (int i = 1; i < 16; ++i) angles.push_back(ang + i * M_PI / 16.0);
  for (const double candidate_angle : angles) {
    auto candidate = GenerateSwaths(candidate_angle, op_width, poly);
    const double candidate_score = coverage.ComputeCost(poly, candidate);
    if (candidate_score > score + 1e-8) {
      swaths = std::move(candidate);
      score = candidate_score;
    }
    if (score >= kCoverageTarget) break;
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
  if (!std::isfinite(op_width) || op_width <= 0.0) {
    throw std::invalid_argument("swath width must be positive and finite");
  }
  const F2CPoint origin = poly.GetExteriorRing().StartPoint();
  const F2CPoint zero(0.0, 0.0);
  auto rot_poly = zero.RotateFromPoint(-angle, poly - origin);

  double field_height {rot_poly.GetHeight()};
  F2CPoint min_point(rot_poly.GetDimMinX(), rot_poly.GetDimMinY());
  auto seed_curve = rot_poly.CreateStraightLongLine(min_point, 0.0);

  // A fixed half-width origin can leave a long uncovered band on a narrow
  // arm, even though every row is exactly one implement width apart. Score a
  // small deterministic set of grid phases against the actual polygon.
  // This optimizes real footprint coverage without changing row spacing.
  obj::FieldCoverage coverage;
  F2CSwaths best;
  double best_coverage = -1.0;
  for (const double phase : {0.5, 0.125, 0.375, 0.625, 0.875}) {
    double curve_y = (phase - 1.0) * op_width;
    F2CMultiLineString paths;
    while (field_height > curve_y + (allow_overlap ? 0.0 : 0.5 * op_width)) {
      curve_y += op_width;
      paths.AddGeometry(seed_curve + F2CPoint(0.0, curve_y));
    }
    F2CSwaths candidate;
    candidate.Append(paths, rot_poly, op_width, types::SwathType::MAINLAND);
    const double score = coverage.ComputeCost(rot_poly, candidate);
    if (score > best_coverage + 1e-8) {
      best = std::move(candidate);
      best_coverage = score;
    }
  }
  for (auto& swath : best) {
    swath.SetPath(zero.RotateFromPoint(angle, swath.GetPath()) + origin);
  }
  return best;
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

