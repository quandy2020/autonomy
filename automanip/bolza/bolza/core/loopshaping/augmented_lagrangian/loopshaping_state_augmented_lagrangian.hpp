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
 * @file loopshaping_state_augmented_lagrangian.hpp
 * @brief Loopshaping state-only augmented Lagrangian collection class.
 */


#include <memory>

#include <bolza/core/augmented_lagrangian/state_augmented_lagrangian_collection.hpp>
#include <bolza/core/loopshaping/loopshaping_definition.hpp>

namespace bolza {

/**
 * Loopshaping state-only augmented Lagrangian collection class
 */
class LoopshapingStateAugmentedLagrangian final : public StateAugmentedLagrangianCollection {
 public:
  /**
   * @brief Constructs a LoopshapingStateAugmentedLagrangian.
   * @param [in] lagrangianCollection: Lagrangian Collection.
   */
  LoopshapingStateAugmentedLagrangian(const StateAugmentedLagrangianCollection& lagrangianCollection,
                                      std::shared_ptr<LoopshapingDefinition> loopshapingDefinition)
      /**
       * @brief State Augmented Lagrangian Collection.
       * @return Result value.
       */
      : StateAugmentedLagrangianCollection(lagrangianCollection), loopshapingDefinition_(std::move(loopshapingDefinition)) {}

  /**
   * @brief Destructor.
   */
  ~LoopshapingStateAugmentedLagrangian() override = default;
  /**
   * @brief Clone.
   * @return Pointer; may be null.
   */
  LoopshapingStateAugmentedLagrangian* Clone() const override { /**
  * @brief Loopshaping State Augmented Lagrangian.
  * @param [in,out] this: This.
  * @return Result value.
  */
 return new LoopshapingStateAugmentedLagrangian(*this); }

  /**
   * @brief Get Value.
   * @param [in] t: T.
   * @param [in] x: X.
   * @param [in] termsMultiplier: Terms Multiplier.
   * @param [in] preComp: Pre Comp.
   * @return Result value.
   */
  std::vector<LagrangianMetrics> GetValue(scalar_t t, const vector_t& x, const std::vector<Multiplier>& termsMultiplier,
                                          const PreComputation& preComp) const override;

  /**
   * @brief Get Quadratic Approximation.
   * @param [in] t: T.
   * @param [in] x: X.
   * @param [in] termsMultiplier: Terms Multiplier.
   * @param [in] preComp: Pre Comp.
   * @return Result value.
   */
  ScalarFunctionQuadraticApproximation GetQuadraticApproximation(scalar_t t, const vector_t& x,
                                                                 const std::vector<Multiplier>& termsMultiplier,
                                                                 const PreComputation& preComp) const override;

  /**
   * @brief Update Lagrangian.
   * @param [in] t: T.
   * @param [in] x: X.
   * @param [in,out] termsMetrics: Terms Metrics.
   * @param [in,out] termsMultiplier: Terms Multiplier.
   */
  void UpdateLagrangian(scalar_t t, const vector_t& x, std::vector<LagrangianMetrics>& termsMetrics,
                        std::vector<Multiplier>& termsMultiplier) const override;

 private:
  /**
   * @brief Constructs a LoopshapingStateAugmentedLagrangian.
   * @param [in] other: Other.
   */
  LoopshapingStateAugmentedLagrangian(const LoopshapingStateAugmentedLagrangian& other) = default;

  std::shared_ptr<LoopshapingDefinition> loopshapingDefinition_;
};

}  // namespace bolza
