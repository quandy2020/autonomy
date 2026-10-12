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
 * @file loopshaping_augmented_lagrangian_output_pattern.hpp
 * @brief loopshaping augmented lagrangian output pattern (bolza).
 */


#include <bolza/core/loopshaping/augmented_lagrangian/loopshaping_state_input_augmented_lagrangian.hpp>

namespace bolza {

class LoopshapingAugmentedLagrangianOutputPattern final : public LoopshapingStateInputAugmentedLagrangian {
 public:
  /**
   * @brief Constructs a LoopshapingAugmentedLagrangianOutputPattern.
   * @param [in] lagrangianCollection: Lagrangian Collection.
   */
  LoopshapingAugmentedLagrangianOutputPattern(const StateInputAugmentedLagrangianCollection& lagrangianCollection,
                                              std::shared_ptr<LoopshapingDefinition> loopshapingDefinition)
      /**
       * @brief Loopshaping State Input Augmented Lagrangian.
       * @param [in] lagrangianCollection: Lagrangian Collection.
       * @return Result value.
       */
      : LoopshapingStateInputAugmentedLagrangian(lagrangianCollection, std::move(loopshapingDefinition)) {}

  /**
   * @brief Destructor.
   */
  ~LoopshapingAugmentedLagrangianOutputPattern() override = default;
  /**
   * @brief Clone.
   * @return Pointer; may be null.
   */
  LoopshapingAugmentedLagrangianOutputPattern* Clone() const override { /**
  * @brief Loopshaping Augmented Lagrangian Output Pattern.
  * @param [in,out] this: This.
  * @return Result value.
  */
 return new LoopshapingAugmentedLagrangianOutputPattern(*this); };

  /**
   * @brief Get Quadratic Approximation.
   * @param [in] t: T.
   * @param [in] x: X.
   * @param [in] u: U.
   * @param [in] termsMultiplier: Terms Multiplier.
   * @param [in] preComp: Pre Comp.
   * @return Result value.
   */
  ScalarFunctionQuadraticApproximation GetQuadraticApproximation(scalar_t t, const vector_t& x, const vector_t& u,
                                                                 const std::vector<Multiplier>& termsMultiplier,
                                                                 const PreComputation& preComp) const override;

 private:
  /**
   * @brief Constructs a LoopshapingAugmentedLagrangianOutputPattern.
   * @param [in] obj: Obj.
   */
  LoopshapingAugmentedLagrangianOutputPattern(const LoopshapingAugmentedLagrangianOutputPattern& obj) = default;
};

}  // namespace bolza
