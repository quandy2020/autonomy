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
 * @file loopshaping_soft_constraint.hpp
 * @brief Factory for Loopshaping state-only soft constraint wrapper.
 */


#include <memory>

#include <bolza/core/cost/state_cost_collection.hpp>
#include <bolza/core/cost/state_input_cost_collection.hpp>
#include <bolza/core/loopshaping/loopshaping_definition.hpp>

namespace bolza {
namespace LoopshapingSoftConstraint {

/**
 * @brief Factory for Loopshaping state-only soft constraint wrapper 
 * @param [in] systemSoftConstraint: System Soft Constraint.
 * @param [in] loopshapingDefinition: Loopshaping Definition.
 * @return Owning pointer.
 */
std::unique_ptr<StateCostCollection> Create(const StateCostCollection& systemSoftConstraint,
                                            std::shared_ptr<LoopshapingDefinition> loopshapingDefinition);

/**
 * @brief Factory for Loopshaping state-input soft constraint wrapper 
 * @param [in] systemSoftConstraint: System Soft Constraint.
 * @param [in] loopshapingDefinition: Loopshaping Definition.
 * @return Owning pointer.
 */
std::unique_ptr<StateInputCostCollection> Create(const StateInputCostCollection& systemSoftConstraint,
                                                 std::shared_ptr<LoopshapingDefinition> loopshapingDefinition);

}  // namespace LoopshapingSoftConstraint
}  // namespace bolza
