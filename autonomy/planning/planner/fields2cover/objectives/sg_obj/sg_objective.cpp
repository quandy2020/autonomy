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
 * @file sg_objective.cpp
 * @brief Fields2Cover sg objective implementation.
 *
 * Ported from Fields2Cover (BSD-3, Wageningen University).
 */



#include "autonomy/planning/planner/fields2cover/objectives/sg_obj/sg_objective.hpp"

namespace autonomy::planning::planner::fields2cover::obj {

double SGObjective::ComputeCost(const F2CSwath&) {
  return 0.0;
}

double SGObjective::ComputeCost(const F2CSwaths& swaths) {
  return std::accumulate(swaths.begin(), swaths.end(), 0.0,
      [this] (double init, const auto& s) {
      return init + this->ComputeCost(s);});
}

double SGObjective::ComputeCost(const F2CSwathsByCells& swaths) {
  return std::accumulate(swaths.begin(), swaths.end(), 0.0,
      [this] (double init, const auto& s) {
      return init + this->ComputeCost(s);});
}

double SGObjective::ComputeCost(const F2CCell& cell) {
  return ComputeCost(0.0, 0.0, cell);
}

double SGObjective::ComputeCost(const F2CCells& cells) {
  return std::accumulate(cells.begin(), cells.end(), 0.0,
      [this] (double init, const auto& s) {
      return init + this->ComputeCost(s);});
}

double SGObjective::ComputeCost(
    double ang, double op_width, const F2CCell& cell) {
  return 0.0;
}

double SGObjective::ComputeCost(
    const F2CCell& c, const F2CSwath& s) {
  return ComputeCost(c) + ComputeCost(s);
}

double SGObjective::ComputeCost(
    const F2CCells& c, const F2CSwath& s) {
  return ComputeCost(c) + ComputeCost(s);
}

double SGObjective::ComputeCost(
    const F2CCell& c, const F2CSwaths& swaths) {
  return std::accumulate(swaths.begin(), swaths.end(), 0.0,
    [this, &c] (double init, const auto& s) {return init + ComputeCost(c, s);});
}

double SGObjective::ComputeCost(const F2CCell& c,
    const F2CSwathsByCells& swaths) {
  return std::accumulate(swaths.begin(), swaths.end(), 0.0,
    [this, &c] (double init, const auto& s) {return init + ComputeCost(c, s);});
}


double SGObjective::ComputeCost(
    const F2CCells& c, const F2CSwaths& swaths) {
  return std::accumulate(swaths.begin(), swaths.end(), 0.0,
    [this, &c] (double init, const auto& s) {return init + ComputeCost(c, s);});
}

double SGObjective::ComputeCost(const F2CCells& c,
    const F2CSwathsByCells& swaths) {
  return std::accumulate(swaths.begin(), swaths.end(), 0.0,
    [this, &c] (double init, const auto& s) {return init + ComputeCost(c, s);});
}


}  // namespace autonomy::planning::planner::fields2cover::obj

