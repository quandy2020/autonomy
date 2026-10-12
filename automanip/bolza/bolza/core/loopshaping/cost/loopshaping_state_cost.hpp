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
 * @file loopshaping_state_cost.hpp
 * @brief Loopshaping state-only cost collection class.
 */


#include <memory>

#include <bolza/core/types.hpp>
#include <bolza/core/cost/state_cost_collection.hpp>
#include <bolza/core/loopshaping/loopshaping_definition.hpp>

namespace bolza {

/**
 * Loopshaping state-only cost collection class
 */
class LoopshapingStateCost final : public StateCostCollection {
 public:
  /**
   * @brief Constructs a LoopshapingStateCost.
   * @param [in] systemCost: System Cost.
   */
  LoopshapingStateCost(const StateCostCollection& systemCost, std::shared_ptr<LoopshapingDefinition> loopshapingDefinition)
      /**
       * @brief State Cost Collection.
       * @return Result value.
       */
      : StateCostCollection(systemCost), loopshapingDefinition_(std::move(loopshapingDefinition)) {}

  /**
   * @brief Destructor.
   */
  ~LoopshapingStateCost() override = default;

  /**
   * @brief Clone.
   * @return Pointer; may be null.
   */
  LoopshapingStateCost* Clone() const override { /**
  * @brief Loopshaping State Cost.
  * @param [in,out] this: This.
  * @return Result value.
  */
 return new LoopshapingStateCost(*this); }

  /**
   * @brief Get Value.
   * @param [in] t: T.
   * @param [in] x: X.
   * @param [in] targetTrajectories: Target Trajectories.
   * @param [in] preComp: Pre Comp.
   * @return Result value.
   */
  scalar_t GetValue(scalar_t t, const vector_t& x, const TargetTrajectories& targetTrajectories,
                    const PreComputation& preComp) const override;

  /**
   * @brief Get Quadratic Approximation.
   * @param [in] t: T.
   * @param [in] x: X.
   * @param [in] targetTrajectories: Target Trajectories.
   * @param [in] preComp: Pre Comp.
   * @return Result value.
   */
  ScalarFunctionQuadraticApproximation GetQuadraticApproximation(scalar_t t, const vector_t& x,
                                                                 const TargetTrajectories& targetTrajectories,
                                                                 const PreComputation& preComp) const override;

 private:
  /**
   * @brief Constructs a LoopshapingStateCost.
   * @param [in] other: Other.
   */
  LoopshapingStateCost(const LoopshapingStateCost& other) = default;

  std::shared_ptr<LoopshapingDefinition> loopshapingDefinition_;
};

}  // namespace bolza
