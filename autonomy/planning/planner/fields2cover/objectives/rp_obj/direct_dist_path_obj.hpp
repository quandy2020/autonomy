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
 * @file direct_dist_path_obj.hpp
 * @brief Fields2Cover direct dist path obj.
 *
 * Ported from Fields2Cover (BSD-3, Wageningen University).
 */


#include "autonomy/planning/planner/fields2cover/types.hpp"
#include "autonomy/planning/planner/fields2cover/objectives/rp_obj/rp_objective.hpp"

namespace autonomy::planning::planner::fields2cover::obj {

/**
 * @class autonomy::planning::planner::fields2cover::obj::DirectDistPathObj
 * @brief Path cost function as the straight Distance between points in a route.
 */
class DirectDistPathObj : public RPObjective {
 public:
  using RPObjective::ComputeCost;

  double ComputeCost(const F2CPoint& p1, const F2CPoint& p2) override;
};


}  // namespace autonomy::planning::planner::fields2cover::obj

