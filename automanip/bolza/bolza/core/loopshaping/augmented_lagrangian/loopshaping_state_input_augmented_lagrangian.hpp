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
 * @file loopshaping_state_input_augmented_lagrangian.hpp
 * @brief Loopshaping state-input augmented Lagrangian decorator base class.
 */


#include <memory>

#include <bolza/core/augmented_lagrangian/state_input_augmented_lagrangian_collection.hpp>
#include <bolza/core/loopshaping/loopshaping_definition.hpp>

namespace bolza {

/**
 * Loopshaping state-input augmented Lagrangian decorator base class
 */
class LoopshapingStateInputAugmentedLagrangian : public StateInputAugmentedLagrangianCollection {
 public:
  /**
   * @brief Destructor.
   */
  ~LoopshapingStateInputAugmentedLagrangian() override = default;

  std::vector<LagrangianMetrics> GetValue(scalar_t t, const vector_t& x, const vector_t& u, const std::vector<Multiplier>& termsMultiplier,
                                          const PreComputation& preComp) const final override;

  void UpdateLagrangian(scalar_t t, const vector_t& x, const vector_t& u, std::vector<LagrangianMetrics>& termsMetrics,
                        std::vector<Multiplier>& termsMultiplier) const final override;

 protected:
    /**
   * @brief Constructor 
   * @param [in] lagrangianCollection: Lagrangian Collection.
   */
  LoopshapingStateInputAugmentedLagrangian(const StateInputAugmentedLagrangianCollection& lagrangianCollection,
                                           std::shared_ptr<LoopshapingDefinition> loopshapingDefinition)
      /**
       * @brief State Input Augmented Lagrangian Collection.
       * @return Result value.
       */
      : StateInputAugmentedLagrangianCollection(lagrangianCollection), loopshapingDefinition_(std::move(loopshapingDefinition)) {}

    /**
   * @brief Copy constructor 
   * @param [in] other: Other.
   */
  LoopshapingStateInputAugmentedLagrangian(const LoopshapingStateInputAugmentedLagrangian& other) = default;

  std::shared_ptr<LoopshapingDefinition> loopshapingDefinition_;
};

}  // namespace bolza
