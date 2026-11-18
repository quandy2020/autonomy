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

#include <type_traits>

/**
 * @file complete_turn_path_obj.hpp
 * @brief Fields2Cover complete turn path obj.
 *
 * Ported from Fields2Cover (BSD-3, Wageningen University).
 */


#include "autonomy/planning/planner/fields2cover/types.hpp"
#include "autonomy/planning/planner/fields2cover/objectives/pp_obj/pp_objective.hpp"
#include "autonomy/planning/planner/fields2cover/objectives/rp_obj/rp_objective.hpp"
#include "autonomy/planning/planner/fields2cover/objectives/rp_obj/direct_dist_path_obj.hpp"
#include "autonomy/planning/planner/fields2cover/path_planning/turning_base.hpp"

namespace autonomy::planning::planner::fields2cover::obj {

/**
 * @class autonomy::planning::planner::fields2cover::obj::CompleteTurnPathObj
 * @brief Cost of turning from one pose to another using a turn planner.
 *
 * This is the closest result to the real cost of executing the path
 * plan, as it actually computes all the turns.
 *
 * @tparam T Turning planner derived from @p pp::TurningBase.
 * @tparam R Path-planning objective used to score the generated turn.
 * @warning Do not use this objective function with slow planners as
 * too many turns may need to be computed for a simple path.
 */
template <class T, class R = PPObjective>
class CompleteTurnPathObj : public RPObjective {
  static_assert(std::is_base_of<pp::TurningBase, T>::value,
      "T must derive from autonomy::planning::planner::fields2cover::pp::TurningBase");
  static_assert(std::is_base_of<PPObjective, R>::value,
      "R must derive from autonomy::planning::planner::fields2cover::obj::PPObjective");

 public:
  using RPObjective::RPObjective;
  explicit CompleteTurnPathObj(const F2CRobot& params);
  explicit CompleteTurnPathObj(const F2CRobot& params, const T& turn_planner);

 public:
  using RPObjective::ComputeCost;

  /**
   * @brief Compute the cost of the turn from @p p1/@p ang1 to @p p2/@p ang2.
   * @param[in] p1 Start point.
   * @param[in] ang1 Start heading.
   * @param[in] p2 End point.
   * @param[in] ang2 End heading.
   * @return Turn cost according to @p R.
   */
  double ComputeCost(
      const F2CPoint& p1, double ang1,
      const F2CPoint& p2, double ang2) override;

  /**
   * @brief Store the robot used when generating turns.
   * @param[in] robot Robot kinematic limits.
   */
  void SetRobot(const F2CRobot& robot);

  /**
   * @brief Store the turn planner used when scoring connections.
   * @param[in] turner Turning planner instance.
   */
  void SetTurnPlanner(const T& turner);

 private:
  /**
   * @brief Planner that derives from autonomy::planning::planner::fields2cover::pp::TurningBase.
   */
  T turn_planner;
  F2CRobot robot;
  R pp_objective;
};


template <class T, class R>
CompleteTurnPathObj<T, R>::CompleteTurnPathObj(const F2CRobot& robot) {
  this->SetRobot(robot);
}

template <class T, class R>
CompleteTurnPathObj<T, R>::CompleteTurnPathObj(
    const F2CRobot& _robot, const T& _turner) :
        turn_planner(_turner), robot(_robot) {}

template <class T, class R>
void CompleteTurnPathObj<T, R>::SetRobot(const F2CRobot& params) {
  this->robot = params;
}

template <class T, class R>
void CompleteTurnPathObj<T, R>::SetTurnPlanner(const T& turner) {
  this->turn_planner = turner;
}


template <class T, class R>
double CompleteTurnPathObj<T, R>::ComputeCost(
    const F2CPoint& p1, double ang1,
    const F2CPoint& p2, double ang2) {
  return pp_objective.ComputeCost(
      this->turn_planner.CreateTurn(this->robot, p1, ang1, p2, ang2));
}


}  // namespace autonomy::planning::planner::fields2cover::obj

