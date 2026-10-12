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
 * @file initialization.hpp
 * @brief Uses the system initializer to determine a state-input initialization at a intermediate node.
 */


#include <bolza/core/types.hpp>
#include <bolza/core/initialization/initializer.hpp>
#include <bolza/core/misc/linear_interpolation.hpp>

#include "bolza/oc/oc_data/primal_solution.hpp"
#include "bolza/oc/oc_data/time_discretization.hpp"

namespace bolza {
namespace multiple_shooting {

/**
 * Uses the system initializer to determine a state-input initialization at a intermediate node
 *
 * @param [in] initializer: System initializer
 * @param [in] t: Start of the discrete interval
 * @param [in] tNext: End time of te discrete interval
 * @param [in] x: Starting state of the discrete interval
 * @return {u(t), x(tNext)} : input and state transition
 */
inline std::pair<vector_t, vector_t> InitializeIntermediateNode(Initializer& initializer, scalar_t t, scalar_t tNext, const vector_t& x) {
  vector_t input, nextState;
  /**
   * @brief Compute.
   * @param [in] t: T.
   * @param [in] x: X.
   * @param [in] tNext: T Next.
   * @param [in] input: Input.
   * @param [in] nextState: Next State.
   * @return Result value.
   */
  initializer.compute(t, x, tNext, input, nextState);
  return {input, nextState};
}

/**
 * Interpolate a primal solution for state-input initialization at a intermediate node
 *
 * @param [in] primalSolution: previous solution
 * @param [in] t: Start of the discrete interval
 * @param [in] tNext: End time of te discrete interval
 * @return {u(t), x(tNext)} : input and state transition
 */
inline std::pair<vector_t, vector_t> InitializeIntermediateNode(const PrimalSolution& primalSolution, scalar_t t, scalar_t tNext) {
  return {LinearInterpolation::Interpolate(t, primalSolution.timeTrajectory_, primalSolution.inputTrajectory_),
          LinearInterpolation::Interpolate(tNext, primalSolution.timeTrajectory_, primalSolution.stateTrajectory_)};
}

/**
 * Initialize the state jump at an event node.
 *
 * @param [in] t: Time at the event node
 * @param [in] x: Pre-event state
 * @return x_next : Post-event state
 */
inline vector_t InitializeEventNode(scalar_t t, const vector_t& x) {
  // Assume identity map for now
  return x;
}

/**
 * Initializes for the state-input trajectories. It interpolates the primalSolution for the starting intersecting time period and then uses
 * initializer for the tail.
 *
 * @param [in] initState: Initial state
 * @param [in] timeDiscretization: The annotated time trajectory
 * @param [in] primalSolution: previous solution
 * @param [in] initializer: System initializer
 * @param [out] stateTrajectory: The initialized state trajectory
 * @param [out] inputTrajectory: The initialized input trajectory
 */
void InitializeStateInputTrajectories(const vector_t& initState, const std::vector<AnnotatedTime>& timeDiscretization,
                                      const PrimalSolution& primalSolution, Initializer& initializer, vector_array_t& stateTrajectory,
                                      vector_array_t& inputTrajectory);

}  // namespace multiple_shooting
}  // namespace bolza
