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
 * @file state_input_cost_collection.hpp
 * @brief State Input Cost function combining a collection of cost terms.
 */


#include <bolza/core/types.hpp>
#include <bolza/core/misc/collection.hpp>
#include <bolza/core/reference/target_trajectories.hpp>

#include "state_input_cost.hpp"

namespace bolza {

/**
 * State Input Cost function combining a collection of cost terms.
 *
 * This class collects a variable number of cost terms and provides methods to get the
 * summed cost values and quadratic approximations. Each cost term can be accessed through its
 * string name and can be activated or deactivated.
 */
class StateInputCostCollection : public Collection<StateInputCost> {
 public:
  /**
   * @brief Constructs a StateInputCostCollection.
   */
  StateInputCostCollection() = default;
  /**
   * @brief Destructor.
   */
  ~StateInputCostCollection() override = default;
  /**
   * @brief Clone.
   * @return Pointer; may be null.
   */
  StateInputCostCollection* Clone() const override;

    /**
   * @brief Get state-input cost value 
   * @param [in] time: Time.
   * @param [in] state: State.
   * @param [in] input: Input.
   * @param [in] targetTrajectories: Target Trajectories.
   * @param [in] preComp: Pre Comp.
   * @return Result value.
   */
  virtual scalar_t GetValue(scalar_t time, const vector_t& state, const vector_t& input, const TargetTrajectories& targetTrajectories,
                            const PreComputation& preComp) const;

    /**
   * @brief Get state-input cost quadratic approximation 
   * @param [in] time: Time.
   * @param [in] state: State.
   * @param [in] input: Input.
   * @param [in] targetTrajectories: Target Trajectories.
   * @param [in] preComp: Pre Comp.
   * @return Result value.
   */
  virtual ScalarFunctionQuadraticApproximation GetQuadraticApproximation(scalar_t time, const vector_t& state, const vector_t& input,
                                                                         const TargetTrajectories& targetTrajectories,
                                                                         const PreComputation& preComp) const;

 protected:
    /**
   * @brief Copy constructor 
   * @param [in] other: Other.
   */
  StateInputCostCollection(const StateInputCostCollection& other);
};

}  // namespace bolza
