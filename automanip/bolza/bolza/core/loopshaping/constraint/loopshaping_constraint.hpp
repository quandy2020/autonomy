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
 * @file loopshaping_constraint.hpp
 * @brief loopshaping constraint (bolza).
 */


#include <memory>

#include <bolza/core/constraint/state_constraint_collection.hpp>
#include <bolza/core/constraint/state_input_constraint_collection.hpp>
#include <bolza/core/loopshaping/loopshaping_definition.hpp>

namespace bolza {
namespace LoopshapingConstraint {

/**
 * @brief Create.
 * @param [in] systemConstraint: System Constraint.
 * @param [in] loopshapingDefinition: Loopshaping Definition.
 * @return Owning pointer.
 */
std::unique_ptr<StateConstraintCollection> Create(const StateConstraintCollection& systemConstraint,
                                                  std::shared_ptr<LoopshapingDefinition> loopshapingDefinition);

/**
 * @brief Create.
 * @param [in] systemConstraint: System Constraint.
 * @param [in] loopshapingDefinition: Loopshaping Definition.
 * @return Owning pointer.
 */
std::unique_ptr<StateInputConstraintCollection> Create(const StateInputConstraintCollection& systemConstraint,
                                                       std::shared_ptr<LoopshapingDefinition> loopshapingDefinition);

}  // namespace LoopshapingConstraint
}  // namespace bolza
