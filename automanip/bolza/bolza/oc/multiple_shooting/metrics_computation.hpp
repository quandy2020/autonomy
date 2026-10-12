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
 * @file metrics_computation.hpp
 * @brief Compute the Metrics for a single intermediate node.
 */


#include <bolza/core/types.hpp>
#include <bolza/core/integration/sensitivity_integrator.hpp>
#include <bolza/core/model_data/metrics.hpp>

#include "bolza/oc/multiple_shooting/transcription.hpp"
#include "bolza/oc/oc_problem/optimal_control_problem.hpp"

namespace bolza {
namespace multiple_shooting {

/**
 * Compute the Metrics for a single intermediate node.
 * @param [in] transcription: multiple shooting transcription for an intermediate node.
 * @return Metrics for a single intermediate node.
 */
Metrics ComputeMetrics(const Transcription& transcription);

/**
 * Compute the Metrics for the event node.
 * @param [in] transcription: multiple shooting transcription for event node.
 * @return Metrics for a event node.
 */
Metrics ComputeMetrics(const EventTranscription& transcription);

/**
 * Compute the Metrics for the terminal node.
 * @param [in] transcription: multiple shooting transcription for terminal node.
 * @return Metrics for a terminal node.
 */
Metrics ComputeMetrics(const TerminalTranscription& transcription);

/**
 * Compute the Metrics for a single intermediate node.
 * @param [in] optimalControlProblem: Definition of the optimal control problem
 * @param [in] discretizer: Integrator to use for creating the discrete dynamics.
 * @param [in] t: Start of the discrete interval
 * @param [in] dt: Duration of the interval
 * @param [in] x: State at start of the interval
 * @param [in] x_next: State at the end of the interval
 * @param [in] u: Input, taken to be constant across the interval.
 * @return Metrics for a single intermediate node.
 */
Metrics ComputeIntermediateMetrics(OptimalControlProblem& optimalControlProblem, DynamicsDiscretizer& discretizer, scalar_t t, scalar_t dt,
                                   const vector_t& x, const vector_t& x_next, const vector_t& u);

/**
 * Compute the Metrics for the event node.
 * @param [in] optimalControlProblem: Definition of the optimal control problem
 * @param [in] t: Time at the event node
 * @param [in] x: Pre-event state
 * @param [in] x_next: Post-event state
 * @return Metrics for the event node.
 */
Metrics ComputeEventMetrics(OptimalControlProblem& optimalControlProblem, scalar_t t, const vector_t& x, const vector_t& x_next);

/**
 * Compute the Metrics for the terminal node.
 * @param [in] optimalControlProblem: Definition of the optimal control problem
 * @param [in] t: Time at the terminal node
 * @param [in] x: Terminal state
 * @return Metrics for the terminal node.
 */
Metrics ComputeTerminalMetrics(OptimalControlProblem& optimalControlProblem, scalar_t t, const vector_t& x);

}  // namespace multiple_shooting
}  // namespace bolza
