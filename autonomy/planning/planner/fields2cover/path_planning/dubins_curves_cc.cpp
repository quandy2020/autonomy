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
 * @file dubins_curves_cc.cpp
 * @brief Fields2Cover dubins curves cc implementation.
 *
 * Ported from Fields2Cover (BSD-3, Wageningen University).
 */



#include "autonomy/common/math/steering/clothoid/dubins_zero_zero.hpp"
#include "autonomy/common/math/steering/clothoid/dubins.hpp"
#include "autonomy/planning/planner/fields2cover/path_planning/steer_to_path.hpp"
#include "autonomy/planning/planner/fields2cover/path_planning/dubins_curves_cc.hpp"

namespace autonomy::planning::planner::fields2cover::pp {

F2CPath DubinsCurvesCC::CreateSimpleTurn(const F2CRobot& robot,
    double dist_start_pos, double start_angle, double end_angle) {
  using autonomy::common::math::steering::Cc00DubinsStateSpace;
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

  double n = 0.0;
  while (true) {
    Cc00DubinsStateSpace ss(
        robot.GetMaxCurv() / (1+0.05*n),
        robot.GetMaxDiffCurv() / (1+0.2*n),
        discretization,
        true);
    if (loop_detected(ss.GetControls(start, end)) && n <= 20.) {
      n += 1.0;
    } else {
      return SteerStatesToPath(ss.GetPath(start, end),
          robot.GetTurnVel());
    }
  }
}

F2CPath DubinsCurvesCC::CreateConstrainedTurn(const F2CRobot& robot,
    const F2CPoint& start_pos, double start_angle, double start_curvature,
    const F2CPoint& end_pos, double end_angle, double end_curvature,
    const bool fix_loops) {
  using autonomy::common::math::steering::CcDubinsStateSpace;
  using autonomy::common::math::steering::State;
  State start{};
  State end{};
  start.x = start_pos.GetX();
  start.y = start_pos.GetY();
  start.theta = start_angle;
  start.kappa = start_curvature;
  start.d = 0;

  end.x = end_pos.GetX();
  end.y = end_pos.GetY();
  end.theta = end_angle;
  end.kappa = end_curvature;
  end.d = 0;
  double n = 0.0;

  while (true) {
    CcDubinsStateSpace ss(
        robot.GetMaxCurv() / (1 + 0.05 * n),
        robot.GetMaxDiffCurv() / (1 + 0.2 * n),
        discretization,
        true);
    if (fix_loops && loop_detected(ss.GetControls(start, end)) && n <= 20.) {
      n += 1.0;
    } else {
      return SteerStatesToPath(ss.GetPath(start, end),
          robot.GetTurnVel());
    }
  }
}

bool DubinsCurvesCC::HasContinuousCurvature() const {
  return true;
}

}  // namespace autonomy::planning::planner::fields2cover::pp
