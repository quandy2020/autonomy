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
 * @file state_input_augmented_lagrangian_collection.hpp
 * @brief State-input Augmented Lagrangian penalty class combining a collection of constraint terms.
 */


#include <functional>

#include <bolza/core/types.hpp>
#include <bolza/core/misc/collection.hpp>

#include "bolza/core/augmented_lagrangian/state_input_augmented_lagrangian_interface.hpp"

namespace bolza {

/**
 * State-input Augmented Lagrangian penalty class combining a collection of constraint terms.
 *
 * This class collects a variable number of Augmented Lagrangian penalty terms and provides methods to get the
 * summed values and quadratic approximations. Each term can be accessed through its string name.
 */
class StateInputAugmentedLagrangianCollection : public Collection<StateInputAugmentedLagrangianInterface> {
 public:
  /**
   * @brief Constructs a StateInputAugmentedLagrangianCollection.
   */
  StateInputAugmentedLagrangianCollection() = default;
  /**
   * @brief Destructor.
   */
  ~StateInputAugmentedLagrangianCollection() override = default;
  /**
   * @brief Clone.
   * @return Pointer; may be null.
   */
  StateInputAugmentedLagrangianCollection* Clone() const override;

    /**
   * @brief /** Get total number of active constraints. */.
   * @param [in] time: Time.
   * @return Result value.
   */
  size_t GetNumberOfActiveConstraints(scalar_t time) const;

    /**
   * @brief /** Get state constraints and their penalties for each active term */.
   * @param [in] time: Time.
   * @param [in] state: State.
   * @param [in] input: Input.
   * @param [in] termsMultiplier: Terms Multiplier.
   * @param [in] preComp: Pre Comp.
   * @return Result value.
   */
  virtual std::vector<LagrangianMetrics> GetValue(scalar_t time, const vector_t& state, const vector_t& input,
                                                  const std::vector<Multiplier>& termsMultiplier, const PreComputation& preComp) const;

    /**
   * @brief /** Get the sum of state-input Lagrangian penalties quadratic approximation */.
   * @param [in] time: Time.
   * @param [in] state: State.
   * @param [in] input: Input.
   * @param [in] termsMultiplier: Terms Multiplier.
   * @param [in] preComp: Pre Comp.
   * @return Result value.
   */
  virtual ScalarFunctionQuadraticApproximation GetQuadraticApproximation(scalar_t time, const vector_t& state, const vector_t& input,
                                                                         const std::vector<Multiplier>& termsMultiplier,
                                                                         const PreComputation& preComp) const;

    /**
   * @brief /** Update Lagrange/penalty multipliers and the penalty value for each active term. */.
   * @param [in] time: Time.
   * @param [in] state: State.
   * @param [in] input: Input.
   * @param [in,out] termsMetrics: Terms Metrics.
   * @param [in,out] termsMultiplier: Terms Multiplier.
   */
  virtual void UpdateLagrangian(scalar_t time, const vector_t& state, const vector_t& input, std::vector<LagrangianMetrics>& termsMetrics,
                                std::vector<Multiplier>& termsMultiplier) const;

    /**
   * @brief /** Initialize Lagrange/penalty multipliers for each active term. */.
   * @param [in] time: Time.
   * @param [in,out] termsMultiplier: Terms Multiplier.
   */
  void InitializeLagrangian(scalar_t time, std::vector<Multiplier>& termsMultiplier) const;

 protected:
  /**
   * @brief Constructs a StateInputAugmentedLagrangianCollection.
   * @param [in] other: Other.
   */
  StateInputAugmentedLagrangianCollection(const StateInputAugmentedLagrangianCollection& other) = default;
};

}  // namespace bolza
