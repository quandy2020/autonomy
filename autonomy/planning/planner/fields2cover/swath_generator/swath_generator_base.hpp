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
 * @file swath_generator_base.hpp
 * @brief Fields2Cover swath generator base.
 *
 * Ported from Fields2Cover (BSD-3, Wageningen University).
 */


#include <vector>
#include "autonomy/planning/planner/fields2cover/types.hpp"
#include "autonomy/planning/planner/fields2cover/objectives/sg_obj/sg_objective.hpp"

namespace autonomy::planning::planner::fields2cover::sg {

/**
 * @class autonomy::planning::planner::fields2cover::sg::SwathGeneratorBase
 * @brief Base class that fills a polygon with parallel swaths.
 */
class SwathGeneratorBase {
 public:
  /**
   * @brief Whether neighboring swaths may overlap.
   * @return true when overlap is allowed.
   */
  bool GetAllowOverlap() const;

  /**
   * @brief Allow or forbid neighboring swaths to overlap.
   * @param[in] allow_overlap Overlap flag.
   */
  void SetAllowOverlap(bool);

  /**
   * @brief Generate swaths at the angle that minimizes @p obj.
   * @param[in] obj Swath-generation objective.
   * @param[in] op_width Implement width.
   * @param[in] poly Cell to cover.
   * @return Swaths covering @p poly.
   */
  virtual F2CSwaths GenerateBestSwaths(autonomy::planning::planner::fields2cover::obj::SGObjective& obj,
      double op_width, const F2CCell& poly);

  /**
   * @brief Generate best swaths for each cell in @p polys.
   * @param[in] obj Swath-generation objective.
   * @param[in] op_width Implement width.
   * @param[in] polys Cells to cover.
   * @return Swaths grouped by cell.
   */
  virtual F2CSwathsByCells GenerateBestSwaths(autonomy::planning::planner::fields2cover::obj::SGObjective& obj,
      double op_width, const F2CCells& polys);

  /**
   * @brief Generate parallel swaths at a fixed angle.
   * @param[in] angle Swath heading in radians.
   * @param[in] op_width Implement width.
   * @param[in] poly Cell to cover.
   * @return Swaths covering @p poly.
   */
  virtual F2CSwaths GenerateSwaths(double angle,
      double op_width, const F2CCell& poly);

  /**
   * @brief Generate parallel swaths at a fixed angle for each cell.
   * @param[in] angle Swath heading in radians.
   * @param[in] op_width Implement width.
   * @param[in] polys Cells to cover.
   * @return Swaths grouped by cell.
   */
  virtual F2CSwathsByCells GenerateSwaths(double angle,
      double op_width, const F2CCells& polys);

  /**
   * @brief Evaluate @p obj for swaths generated at @p ang.
   * @param[in] obj Swath-generation objective.
   * @param[in] ang Candidate heading in radians.
   * @param[in] op_width Implement width.
   * @param[in] poly Cell to cover.
   * @return Objective cost of that heading.
   */
  virtual double ComputeCostOfAngle(autonomy::planning::planner::fields2cover::obj::SGObjective& obj,
      double ang, double op_width, const F2CCell& poly);

  /**
   * @brief Search for the heading that minimizes @p obj.
   * @param[in] obj Swath-generation objective.
   * @param[in] op_width Implement width.
   * @param[in] poly Cell to cover.
   * @return Best heading in radians.
   */
  virtual double ComputeBestAngle(autonomy::planning::planner::fields2cover::obj::SGObjective& obj,
      double op_width, const F2CCell& poly) = 0;

  /**
   * @brief Destructor.
   */
  virtual ~SwathGeneratorBase() = default;

 protected:
  bool allow_overlap {false};
};

}  // namespace autonomy::planning::planner::fields2cover::sg


