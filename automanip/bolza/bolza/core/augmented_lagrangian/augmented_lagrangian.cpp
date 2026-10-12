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

#include "bolza/core/augmented_lagrangian/augmented_lagrangian.hpp"

namespace bolza {

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
std::unique_ptr<StateAugmentedLagrangian> Create(std::unique_ptr<StateConstraint> constraintPtr,
                                                 std::vector<std::unique_ptr<augmented::AugmentedPenaltyBase>> penaltyPtrArray) {
  return std::make_unique<StateAugmentedLagrangian>(std::move(constraintPtr), std::move(penaltyPtrArray));
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
std::unique_ptr<StateAugmentedLagrangian> Create(std::unique_ptr<StateConstraint> constraintPtr,
                                                 std::unique_ptr<augmented::AugmentedPenaltyBase> penaltyPtr) {
  return std::make_unique<StateAugmentedLagrangian>(std::move(constraintPtr), std::move(penaltyPtr));
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
std::unique_ptr<StateInputAugmentedLagrangian> Create(std::unique_ptr<StateInputConstraint> constraintPtr,
                                                      std::vector<std::unique_ptr<augmented::AugmentedPenaltyBase>> penaltyPtrArray) {
  return std::make_unique<StateInputAugmentedLagrangian>(std::move(constraintPtr), std::move(penaltyPtrArray));
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
std::unique_ptr<StateInputAugmentedLagrangian> Create(std::unique_ptr<StateInputConstraint> constraintPtr,
                                                      std::unique_ptr<augmented::AugmentedPenaltyBase> penaltyPtr) {
  return std::make_unique<StateInputAugmentedLagrangian>(std::move(constraintPtr), std::move(penaltyPtr));
}

}  // namespace bolza
