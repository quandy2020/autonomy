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
 * @file brute_force.cpp
 * @brief Fields2Cover brute force implementation.
 *
 * Ported from Fields2Cover (BSD-3, Wageningen University).
 */



#include <algorithm>
#ifdef ALLOW_PARALLELIZATION
#include <execution>
#endif
#include <vector>
#include <utility>
#include <limits>
#include "autonomy/planning/planner/fields2cover/swath_generator/brute_force.hpp"

namespace autonomy::planning::planner::fields2cover::sg {

double BruteForce::GetStepAngle() const {
  return this->step_angle;
}

void BruteForce::SetStepAngle(double d) {
  this->step_angle = d;
}

double BruteForce::ComputeBestAngle(autonomy::planning::planner::fields2cover::obj::SGObjective& obj,
    double op_width, const F2CCell& poly) {
  int n = static_cast<int>(
      boost::math::constants::two_pi<double>() / step_angle);
  std::vector<double> costs(n);
  std::vector<int> ids(n);
  std::iota(ids.begin(), ids.end(), 0);

  auto GetCostSwaths = [this, op_width, &poly, &obj] (const int& i) {
    return ComputeCostOfAngle(obj, i * step_angle, op_width, poly);
  };

  #ifdef ALLOW_PARALLELIZATION
    std::transform(std::execution::par_unseq, ids.begin(), ids.end(),
        costs.begin(), GetCostSwaths);
  #else
    std::transform(ids.begin(), ids.end(), costs.begin(), GetCostSwaths);
  #endif

  return ids[std::min_element(
      costs.begin(), costs.end()) - costs.begin()] * step_angle;
}

}  // namespace autonomy::planning::planner::fields2cover::sg

