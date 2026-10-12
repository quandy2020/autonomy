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
 * @file loopshaping_cost_eliminate_pattern.hpp
 * @brief loopshaping cost eliminate pattern (bolza).
 */


#include <bolza/core/loopshaping/cost/loopshaping_state_input_cost.hpp>

namespace bolza {

class LoopshapingCostEliminatePattern final : public LoopshapingStateInputCost {
 public:
  using BASE = LoopshapingStateInputCost;

  /**
   * @brief Constructs a LoopshapingCostEliminatePattern.
   * @param [in] systemCost: System Cost.
   */
  LoopshapingCostEliminatePattern(const StateInputCostCollection& systemCost, std::shared_ptr<LoopshapingDefinition> loopshapingDefinition)
      /**
       * @brief BASE.
       * @param [in] systemCost: System Cost.
       * @return Result value.
       */
      : BASE(systemCost, std::move(loopshapingDefinition)) {}

  /**
   * @brief Destructor.
   */
  ~LoopshapingCostEliminatePattern() override = default;

  /**
   * @brief Constructs a LoopshapingCostEliminatePattern.
   * @param [in] obj: Obj.
   */
  LoopshapingCostEliminatePattern(const LoopshapingCostEliminatePattern& obj) = default;

  /**
   * @brief Clone.
   * @return Pointer; may be null.
   */
  LoopshapingCostEliminatePattern* Clone() const override { /**
  * @brief Loopshaping Cost Eliminate Pattern.
  * @param [in,out] this: This.
  * @return Result value.
  */
 return new LoopshapingCostEliminatePattern(*this); };

  /**
   * @brief Get Quadratic Approximation.
   * @param [in] t: T.
   * @param [in] x: X.
   * @param [in] u: U.
   * @param [in] targetTrajectories: Target Trajectories.
   * @param [in] preComp: Pre Comp.
   * @return Result value.
   */
  ScalarFunctionQuadraticApproximation GetQuadraticApproximation(scalar_t t, const vector_t& x, const vector_t& u,
                                                                 const TargetTrajectories& targetTrajectories,
                                                                 const PreComputation& preComp) const override;

 protected:
  using BASE::loopshapingDefinition_;
};

}  // namespace bolza
