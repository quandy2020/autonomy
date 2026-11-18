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
 * @file brute_force.hpp
 * @brief Fields2Cover brute force.
 *
 * Ported from Fields2Cover (BSD-3, Wageningen University).
 */


#include <limits>
#include <utility>
#include <numeric>
#include <memory>
#include "autonomy/planning/planner/fields2cover/types.hpp"
#include "autonomy/planning/planner/fields2cover/swath_generator/swath_generator_base.hpp"

namespace autonomy::planning::planner::fields2cover::sg {

/**
 * @class autonomy::planning::planner::fields2cover::sg::BruteForce
 * @brief Exhaustive search of swath headings at a fixed angular step.
 */
class BruteForce : public SwathGeneratorBase {
 public:
  /**
   * @brief Angular step used by the exhaustive search.
   * @return Step in radians.
   */
  double GetStepAngle() const;

  /**
   * @brief Set the angular step of the exhaustive search.
   * @param[in] d Step in radians.
   */
  void SetStepAngle(double d);

  /**
   * @brief Search headings in [0, pi) and return the lowest-cost angle.
   * @param[in] obj Swath-generation objective.
   * @param[in] op_width Implement width.
   * @param[in] poly Cell to cover.
   * @return Best heading in radians.
   */
  double ComputeBestAngle(autonomy::planning::planner::fields2cover::obj::SGObjective& obj,
      double op_width, const F2CCell& poly) override;

 private:
  double step_angle {boost::math::constants::degree<double>()};  // radians
};


}  // namespace autonomy::planning::planner::fields2cover::sg



