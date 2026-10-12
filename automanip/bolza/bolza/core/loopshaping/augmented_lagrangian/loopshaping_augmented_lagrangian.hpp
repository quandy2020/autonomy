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
 * @file loopshaping_augmented_lagrangian.hpp
 * @brief Factory for Loopshaping state-only augmented Lagrangian wrapper.
 */


#include <memory>

#include <bolza/core/augmented_lagrangian/state_augmented_lagrangian_collection.hpp>
#include <bolza/core/augmented_lagrangian/state_input_augmented_lagrangian_collection.hpp>
#include <bolza/core/loopshaping/loopshaping_definition.hpp>

namespace bolza {
namespace LoopshapingAugmentedLagrangian {

/**
 * @brief /** Factory for Loopshaping state-only augmented Lagrangian wrapper */.
 * @param [in] systemAugmentedLagrangian: System Augmented Lagrangian.
 * @param [in] loopshapingDefinition: Loopshaping Definition.
 * @return Owning pointer.
 */
std::unique_ptr<StateAugmentedLagrangianCollection> Create(const StateAugmentedLagrangianCollection& systemAugmentedLagrangian,
                                                           std::shared_ptr<LoopshapingDefinition> loopshapingDefinition);

/**
 * @brief /** Factory for Loopshaping state-input augmented Lagrangian wrapper */.
 * @param [in] systemAugmentedLagrangian: System Augmented Lagrangian.
 * @param [in] loopshapingDefinition: Loopshaping Definition.
 * @return Owning pointer.
 */
std::unique_ptr<StateInputAugmentedLagrangianCollection> Create(const StateInputAugmentedLagrangianCollection& systemAugmentedLagrangian,
                                                                std::shared_ptr<LoopshapingDefinition> loopshapingDefinition);

}  // namespace LoopshapingAugmentedLagrangian
}  // namespace bolza
