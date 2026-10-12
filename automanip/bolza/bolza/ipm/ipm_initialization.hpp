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
 * @file ipm_initialization.hpp
 * @brief Initializes the slack variable. The initialization of the slack variables follows IPOPT.
 */


#include <bolza/core/types.hpp>
#include <bolza/oc/oc_problem/optimal_control_problem.hpp>

namespace bolza {
namespace ipm {

/**
 * Initializes the slack variable. The initialization of the slack variables follows IPOPT
 * (https://coin-or.github.io/Ipopt/OPTIONS.html#OPT_Initialization).
 *
 * @param [in] ineqConstraint: The value of the inequality constraint.
 * @param [in] initialSlackLowerBound: Lower bound of the initial slack variables. Corresponds to `slack_bound_push` option of IPOPT.
 * @param [in] initialSlackMarginRate: Additional margin rate of the initial slack variables. Corresponds to `slack_bound_frac` option of IPOPT.
 * @return Initialized slack variable.
 */
inline vector_t InitializeSlackVariable(const vector_t& ineqConstraint, scalar_t initialSlackLowerBound, scalar_t initialSlackMarginRate) {
  if (ineqConstraint.size() > 0) {
    return (1.0 + initialSlackMarginRate) * ineqConstraint.cwiseMax(initialSlackLowerBound);
  } else {
    /**
     * @brief Vector t.
     * @return Result value.
     */
    return vector_t();
  }
}

/**
 * Initializes the dual variable. The initialization of dual variables follows IPOPT option `bound_mult_init_method`: `mu-based`
 * (https://coin-or.github.io/Ipopt/OPTIONS.html#OPT_Initialization) with additoinal options.
 *
 * @param [in] slack: The slack variable.
 * @param [in] barrierParam: The barrier parameter of the IPM. Must be positive.
 * @param [in] initialDualLowerBound: Lower bound of the initial dual variables.
 * @param [in] initialDualMarginRate: Additional margin rate of the initial dual variables.
 * @return Initialized dual variable.
 */
inline vector_t InitializeDualVariable(const vector_t& slack, scalar_t barrierParam, scalar_t initialDualLowerBound,
                                       scalar_t initialDualMarginRate) {
  if (slack.size() > 0) {
    return (1.0 + initialDualMarginRate) * (barrierParam * slack.cwiseInverse()).cwiseMax(initialDualLowerBound);
  } else {
    /**
     * @brief Vector t.
     * @return Result value.
     */
    return vector_t();
  }
}

/**
 * Initializes the slack variable at a single intermediate node.
 *
 * @param [in] ocpDefinition: Definition of the optimal control problem.
 * @param [in] time: Time of this node
 * @param [in] state: State
 * @param [in] input: Input
 * @param [in] initialSlackLowerBound: Lower bound of the initial slack variables. Corresponds to `slack_bound_push` option of IPOPT.
 * @param [in] initialSlackMarginRate: Additional margin rate of the initial slack variables. Corresponds to `slack_bound_frac` option of IPOPT.
 * @return Initialized slack variables of the intermediate state-only (first) and state-input (second) constraints.
 */
std::pair<vector_t, vector_t> InitializeIntermediateSlackVariable(OptimalControlProblem& ocpDefinition, scalar_t time,
                                                                  const vector_t& state, const vector_t& input,
                                                                  scalar_t initialSlackLowerBound, scalar_t initialSlackMarginRate);

/**
 * Initializes the slack variable at the terminal node.
 *
 * @param [in] ocpDefinition: Definition of the optimal control problem.
 * @param [in] time: Time at the terminal node
 * @param [in] state: Terminal state
 * @param [in] initialSlackLowerBound: Lower bound of the initial slack variables. Corresponds to `slack_bound_push` option of IPOPT.
 * @param [in] initialSlackMarginRate: Additional margin rate of the initial slack variables. Corresponds to `slack_bound_frac` option of IPOPT.
 * @return Initialized slack variable of the terminal state-only constraints.
 */
vector_t InitializeTerminalSlackVariable(OptimalControlProblem& ocpDefinition, scalar_t time, const vector_t& state,
                                         scalar_t initialSlackLowerBound, scalar_t initialSlackMarginRate);

/**
 * Initializes the slack variable at an event node.
 *
 * @param [in] ocpDefinition: Definition of the optimal control problem.
 * @param [in] time: Time at the event node
 * @param [in] state: Pre-event state
 * @param [in] initialSlackLowerBound: Lower bound of the initial slack variables. Corresponds to `slack_bound_push` option of IPOPT.
 * @param [in] initialSlackMarginRate: Additional margin rate of the initial slack variables. Corresponds to `slack_bound_frac` option of IPOPT.
 * @return Initialized slack variable of the pre-jump state-only constraints.
 */
vector_t InitializeEventSlackVariable(OptimalControlProblem& ocpDefinition, scalar_t time, const vector_t& state,
                                      scalar_t initialSlackLowerBound, scalar_t initialSlackMarginRate);

}  // namespace ipm
}  // namespace bolza