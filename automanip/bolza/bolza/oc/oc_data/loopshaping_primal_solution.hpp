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
 * @file loopshaping_primal_solution.hpp
 * @brief Converts a primal solution obtained for loopshaping wrapped system to the primal solution of the original system.
 */


#include <bolza/core/loopshaping/loopshaping_definition.hpp>

#include "bolza/oc/oc_data/primal_solution.hpp"

namespace bolza {

/**
 * Converts a primal solution obtained for loopshaping wrapped system to the primal solution of the original system.
 * The controller is not converted and set to a nullptr.
 *
 * @param [in] primalSolution: solution for the augmented (=loopshaped) system
 * @param [in] loopshapingDefinition: loopshaping filter definition
 * @return primal solution with state and inputs of the original system.
 */
PrimalSolution LoopshapingToSystemPrimalSolution(const PrimalSolution& primalSolution, const LoopshapingDefinition& loopshapingDefinition);

}  // namespace bolza
