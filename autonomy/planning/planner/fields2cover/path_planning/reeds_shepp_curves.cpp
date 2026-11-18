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
 * @file reeds_shepp_curves.cpp
 * @brief Fields2Cover reeds shepp curves implementation.
 *
 * Ported from Fields2Cover (BSD-3, Wageningen University).
 */



#include "autonomy/common/math/steering/reeds_shepp.hpp"
#include "autonomy/planning/planner/fields2cover/path_planning/steer_to_path.hpp"
#include "autonomy/planning/planner/fields2cover/path_planning/reeds_shepp_curves.hpp"

namespace autonomy::planning::planner::fields2cover::pp {

F2CPath ReedsSheppCurves::CreateSimpleTurn(const F2CRobot& robot,
    double dist_start_pos, double start_angle, double end_angle) {
  using autonomy::common::math::steering::ReedsSheppStateSpace;
  using autonomy::common::math::steering::State;
  State start{};
  State end{};

  start.x = 0.0;
  start.y = 0.0;
  start.theta = start_angle;
  start.kappa = 0.0;
  start.d = 0;

  end.x = dist_start_pos;
  end.y = 0.0;
  end.theta = end_angle;
  end.kappa = 0.0;
  end.d = 0;

  ReedsSheppStateSpace ss(
      robot.GetMaxCurv(),
      discretization);
  return SteerStatesToPath(ss.GetPath(start, end),
      robot.GetTurnVel());
}

}  // namespace autonomy::planning::planner::fields2cover::pp

