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
 * @file reeds_shepp_curves.hpp
 * @brief Fields2Cover reeds shepp curves.
 *
 * Ported from Fields2Cover (BSD-3, Wageningen University).
 */


#include "autonomy/planning/planner/fields2cover/types.hpp"
#include "autonomy/planning/planner/fields2cover/path_planning/turning_base.hpp"

namespace autonomy::planning::planner::fields2cover::pp {

/**
 * @class autonomy::planning::planner::fields2cover::pp::ReedsSheppCurves
 * @brief Reeds-Shepp's curves planner.
 */
class ReedsSheppCurves : public TurningBase {
 public:
  /**
   * @brief Plan a Reeds-Shepp turn (forward and reverse).
   * @param[in] robot Robot kinematic limits.
   * @param[in] dist_start_pos Distance between start and end points.
   * @param[in] start_angle Incoming heading.
   * @param[in] end_angle Outgoing heading.
   * @return Discretized turn path.
   */
  F2CPath CreateSimpleTurn(const F2CRobot& robot,
      double dist_start_pos, double start_angle, double end_angle) override;
};

}  // namespace autonomy::planning::planner::fields2cover::pp

