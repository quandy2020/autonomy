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
 * @file path_planning.hpp
 * @brief Fields2Cover path planning.
 *
 * Ported from Fields2Cover (BSD-3, Wageningen University).
 */


#include <utility>
#include <vector>
#include "autonomy/planning/planner/fields2cover/types.hpp"
#include "autonomy/planning/planner/fields2cover/path_planning/turning_base.hpp"

namespace autonomy::planning::planner::fields2cover::pp {

/**
 * @class autonomy::planning::planner::fields2cover::pp::PathPlanning
 * @brief Path planning class to connect a path using a TurningBase class method.
 */
class PathPlanning {
 public:
  /**
   * @brief Compute the coverage path using a route as a reference.
   * @param[in] robot Robot kinematic limits.
   * @param[in] route Coverage route of swaths and connections.
   * @param[in,out] turn Turn planner used between swaths.
   * @return Coverage path including turns.
   */
  static F2CPath PlanPath(const F2CRobot& robot, const F2CRoute& route,
      TurningBase& turn);

  /**
   * @brief Compute a path that covers the swaths in order,
   * using a path planner to connect swaths.
   * @param[in] robot Robot kinematic limits.
   * @param[in] swaths Swaths to cover in given order.
   * @param[in,out] turn Turn planner used between swaths.
   * @return Coverage path including turns.
   */
  static F2CPath PlanPath(const F2CRobot& robot, const F2CSwaths& swaths,
      TurningBase& turn);

  /**
   * @brief Compute a path from the end of @p s1 to the start of @p s2
   * through optional middle points.
   * @param[in] robot Robot kinematic limits.
   * @param[in] s1 First swath group.
   * @param[in] mp Optional intermediate waypoints.
   * @param[in] s2 Second swath group.
   * @param[in,out] turn Turn planner used for the connection.
   * @return Connection path.
   */
  static F2CPath PlanPathForConnection(const F2CRobot& robot,
      const F2CSwaths& s1, const F2CMultiPoint& mp, const F2CSwaths& s2,
      TurningBase& turn, const F2CCells* field = nullptr);

  /**
   * @brief Compute a path that connects two poses and visits middle points.
   * @param[in] robot Robot kinematic limits.
   * @param[in] p1 Start point.
   * @param[in] ang1 Start heading.
   * @param[in] mp Intermediate waypoints.
   * @param[in] p2 End point.
   * @param[in] ang2 End heading.
   * @param[in,out] turn Turn planner used for the connection.
   * @return Connection path.
   */
  static F2CPath PlanPathForConnection(const F2CRobot& robot,
      const F2CPoint& p1, double ang1,
      const F2CMultiPoint& mp,
      const F2CPoint& p2, double ang2,
      TurningBase& turn, const F2CCells* field = nullptr);

  /**
   * @brief Radius swept by a continuous-curvature turn.
   *
   * Wider than the minimum turning radius, as the clothoid ramps spend part
   * of the deflection.
   * @param[in] robot Robot kinematic limits.
   * @return Smooth turning radius in meters.
   */
  static double GetSmoothTurningRadius(const F2CRobot& robot);
};

}  // namespace autonomy::planning::planner::fields2cover::pp

