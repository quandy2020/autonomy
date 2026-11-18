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
 * @file sg_objective.hpp
 * @brief Fields2Cover sg objective.
 *
 * Ported from Fields2Cover (BSD-3, Wageningen University).
 */


#include "autonomy/planning/planner/fields2cover/types.hpp"
#include "autonomy/planning/planner/fields2cover/objectives/base_objective.hpp"


namespace autonomy::planning::planner::fields2cover::obj {

/**
 * @class autonomy::planning::planner::fields2cover::obj::SGObjective
 * @brief Base class for objective functions of swath Generators.
 *
 * SG objectives are those objectives that depends on the swaths Generation
 * Some of the could be autonomy::planning::planner::fields2cover::obj::NSwath or autonomy::planning::planner::fields2cover::obj::FieldCoverage
 */
class SGObjective : public BaseObjective<SGObjective> {
 public:
  virtual bool IsFastCompAvailable() const {return false;}

  /**
   * @brief Compute the cost function.
   */
  virtual double ComputeCost(const F2CSwath&);
  /**
   * @brief Compute the cost function.
   */
  virtual double ComputeCost(const F2CSwaths&);
  /**
   * @brief Compute the cost function.
   */
  virtual double ComputeCost(const F2CSwathsByCells& swaths);
  /**
   * @brief Compute the cost function.
   */
  virtual double ComputeCost(const F2CCell&);
  /**
   * @brief Compute the cost function.
   */
  virtual double ComputeCost(const F2CCells&);
  /**
   * @brief Compute the cost function.
   */
  virtual double ComputeCost(double ang, double op_width, const F2CCell& cell);
  /**
   * @brief Compute the cost function.
   */
  virtual double ComputeCost(const F2CCell& c, const F2CSwath& s);
  /**
   * @brief Compute the cost function.
   */
  virtual double ComputeCost(const F2CCell& c, const F2CSwaths& s);
  /**
   * @brief Compute the cost function.
   */
  virtual double ComputeCost(const F2CCell& c,
      const F2CSwathsByCells& swaths);
  /**
   * @brief Compute the cost function.
   */
  virtual double ComputeCost(const F2CCells& c, const F2CSwath& s);
  /**
   * @brief Compute the cost function.
   */
  virtual double ComputeCost(const F2CCells& c, const F2CSwaths& s);
  /**
   * @brief Compute the cost function.
   */
  virtual double ComputeCost(const F2CCells& c,
      const F2CSwathsByCells& swaths);
};


}  // namespace autonomy::planning::planner::fields2cover::obj

