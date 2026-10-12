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
 * @file loopshaping_optimal_control_problem.hpp
 * @brief loopshaping optimal control problem (bolza).
 */


#include <bolza/core/loopshaping/loopshaping_definition.hpp>
#include <bolza/oc/oc_problem/optimal_control_problem.hpp>

namespace bolza {
namespace LoopshapingOptimalControlProblem {

/**
 * @brief Create.
 * @param [in] problem: Problem.
 * @param [in] loopshapingDefinition: Loopshaping Definition.
 * @return Result value.
 */
OptimalControlProblem Create(const OptimalControlProblem& problem, std::shared_ptr<LoopshapingDefinition> loopshapingDefinition);

}
}  // namespace bolza
