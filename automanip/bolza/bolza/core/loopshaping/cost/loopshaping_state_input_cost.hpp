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
 * @file loopshaping_state_input_cost.hpp
 * @brief Loopshaping state-input cost collection base class.
 */


#include <memory>

#include <bolza/core/types.hpp>
#include <bolza/core/cost/state_input_cost_collection.hpp>
#include <bolza/core/loopshaping/loopshaping_definition.hpp>

namespace bolza {

/**
 * Loopshaping state-input cost collection base class
 */
class LoopshapingStateInputCost : public StateInputCostCollection {
 public:
  /**
   * @brief Destructor.
   */
  ~LoopshapingStateInputCost() override = default;

  /**
   * @brief Get Value.
   * @param [in] t: T.
   * @param [in] x: X.
   * @param [in] u: U.
   * @param [in] targetTrajectories: Target Trajectories.
   * @param [in] preComp: Pre Comp.
   * @return Result value.
   */
  scalar_t GetValue(scalar_t t, const vector_t& x, const vector_t& u, const TargetTrajectories& targetTrajectories,
                    const PreComputation& preComp) const final;

 protected:
    /**
   * @brief /** Constructor */.
   * @param [in] systemCost: System Cost.
   */
  LoopshapingStateInputCost(const StateInputCostCollection& systemCost, std::shared_ptr<LoopshapingDefinition> loopshapingDefinition)
      /**
       * @brief State Input Cost Collection.
       * @return Result value.
       */
      : StateInputCostCollection(systemCost), loopshapingDefinition_(std::move(loopshapingDefinition)) {}

    /**
   * @brief /** Copy constructor */.
   * @param [in] other: Other.
   */
  LoopshapingStateInputCost(const LoopshapingStateInputCost& other) = default;

  std::shared_ptr<LoopshapingDefinition> loopshapingDefinition_;
};

}  // namespace bolza
