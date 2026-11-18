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
 * @file steer_to_path.hpp
 * @brief Fields2Cover steer to path.
 *
 * Ported from Fields2Cover (BSD-3, Wageningen University).
 */


#include <vector>
#include "autonomy/common/math/steering/types.hpp"
#include "autonomy/planning/planner/fields2cover/types.hpp"


namespace autonomy::planning::planner::fields2cover::pp {

namespace steer = autonomy::common::math::steering;

/**
 * @brief Convert steering-library states into a Fields2Cover path.
 * @param[in] curve Sampled steer states along the turn.
 * @param[in] const_vel Constant linear speed assigned to each state.
 * @return Path whose states copy @p curve and use TURN section type.
 */
inline types::Path SteerStatesToPath(
    const std::vector<steer::State>& curve, double const_vel) {
  types::Path path;
  auto compute_dist = [&curve](int i) {
    return ((i + 1 < curve.size()) ?
        F2CPoint(curve[i].x, curve[i].y).Distance(
        F2CPoint(curve[i + 1].x, curve[i + 1].y)) : 0);
  };
  for (size_t i = 0; i < curve.size(); ++i) {
    autonomy::planning::planner::fields2cover::types::PathState state;
    state.point = F2CPoint(curve[i].x, curve[i].y);
    state.angle = curve[i].theta;
    state.velocity = const_vel;
    state.len = compute_dist(i);
    state.dir = static_cast<types::PathDirection>(curve[i].d);
    state.type = types::PathSectionType::TURN;
    path.AddState(state);
  }
  return path;
}

/**
 * @brief Detect a self-overlapping clothoid / arc sequence.
 * @param[in] controls Steering primitives that make up the candidate turn.
 * @return true when the controls wind far enough to form a loop.
 *
 * Contributed by Phact (https://phact.nl/).
 */
inline bool loop_detected(
    const std::vector<autonomy::common::math::steering::Control>& controls) {
  double DrTotal = 0;
  double DrAbsTotal = 0;
  for (auto&& c : controls) {
    if (c.kappa != 0) {
      double dr = 0.5 * c.delta_s * c.kappa / M_PI;
      if (fabs(dr) > 0.9) {
        return true;
      }
      DrTotal += dr;
      DrAbsTotal += fabs(dr);
    }
  }
  return (fabs(DrTotal) > 0.9) || (DrAbsTotal > 1.5);
}
// Function loop_detected contributed by Phact (https://phact.nl/) company



}  // namespace autonomy::planning::planner::fields2cover::pp

