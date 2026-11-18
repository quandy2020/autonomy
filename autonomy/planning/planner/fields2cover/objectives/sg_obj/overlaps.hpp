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

#pragma once

/**
 * @file overlaps.hpp
 * @brief Fields2Cover overlaps.
 *
 * Ported from Fields2Cover (BSD-3, Wageningen University).
 */


#include <utility>
#include <vector>
#include "autonomy/planning/planner/fields2cover/types.hpp"
#include "autonomy/planning/planner/fields2cover/objectives/sg_obj/sg_objective.hpp"

namespace autonomy::planning::planner::fields2cover::obj {

/**
 * @class autonomy::planning::planner::fields2cover::obj::Overlaps
 * @brief SG cost function as the percentage of the field overlapped.
 */
class Overlaps : public SGObjective {
 public:
  using SGObjective::ComputeCost;

  double ComputeCost(const F2CCell& poly, const F2CSwaths& swaths) override;
  double ComputeCost(const F2CCells& poly, const F2CSwaths& swaths) override;
};

}  // namespace autonomy::planning::planner::fields2cover::obj

