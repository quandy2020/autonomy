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
 * @file augmented_lagrangian.hpp
 * @brief Helper factory functions.
 */


#include <memory>

#include <bolza/core/augmented_lagrangian/state_augmented_lagrangian.hpp>
#include <bolza/core/augmented_lagrangian/state_input_augmented_lagrangian.hpp>

/**
 * Helper factory functions
 */
namespace bolza {

/**
 * @brief /**.
 *
 * Factory function for state augmented Lagrangian.
 *
 * /
 * @param [in] constraintPtr: A pointer to the constraint which will be enforced as soft constraints.
 * @param [in] penaltyPtrArray: An array of pointers to the penalty function on the constraint.
 * @return Owning pointer.
 */
std::unique_ptr<StateAugmentedLagrangian> Create(std::unique_ptr<StateConstraint> constraintPtr,
                                                 std::vector<std::unique_ptr<augmented::AugmentedPenaltyBase>> penaltyPtrArray);

/**
 * @brief /**.
 *
 * Factory function for state augmented Lagrangian.
 *
 * /
 * @param [in] constraintPtr: A pointer to the constraint which will be enforced as soft constraints.
 * @param [in] penaltyPtr: A pointer to the penalty function on the constraint.
 * @return Owning pointer.
 */
std::unique_ptr<StateAugmentedLagrangian> Create(std::unique_ptr<StateConstraint> constraintPtr,
                                                 std::unique_ptr<augmented::AugmentedPenaltyBase> penaltyPtr);

/**
 * @brief /**.
 *
 * Factory function for state-input augmented Lagrangian.
 *
 * /
 * @param [in] constraintPtr: A pointer to the constraint which will be enforced as soft constraints.
 * @param [in] penaltyPtrArray: An array of pointers to the penalty function on the constraint.
 * @return Owning pointer.
 */
std::unique_ptr<StateInputAugmentedLagrangian> Create(std::unique_ptr<StateInputConstraint> constraintPtr,
                                                      std::vector<std::unique_ptr<augmented::AugmentedPenaltyBase>> penaltyPtrArray);

/**
 * @brief /**.
 *
 * Factory function for state-input augmented Lagrangian.
 *
 * /
 * @param [in] constraintPtr: A pointer to the constraint which will be enforced as soft constraints.
 * @param [in] penaltyPtr: A pointer to the penalty function on the constraint.
 * @return Owning pointer.
 */
std::unique_ptr<StateInputAugmentedLagrangian> Create(std::unique_ptr<StateInputConstraint> constraintPtr,
                                                      std::unique_ptr<augmented::AugmentedPenaltyBase> penaltyPtr);

}  // namespace bolza
