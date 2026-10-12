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
 * @file performance_index_computation.hpp
 * @brief Compute the performance index from the transcription for an intermediate node.
 */


#include <bolza/core/types.hpp>
#include <bolza/core/integration/sensitivity_integrator.hpp>

#include "bolza/oc/multiple_shooting/transcription.hpp"
#include "bolza/oc/oc_data/performance_index.hpp"
#include "bolza/oc/oc_problem/optimal_control_problem.hpp"

namespace bolza {
namespace multiple_shooting {

/**
 * Compute the performance index from the transcription for an intermediate node.
 * @param [in] transcription: multiple shooting transcription for an intermediate node.
 * @param [in] dt: Duration of the interval
 * @return Performance index for a single intermediate node.
 */
PerformanceIndex ComputePerformanceIndex(const Transcription& transcription, scalar_t dt);

/**
 * Compute the performance index from the transcription for the event node.
 * @param [in] transcription: multiple shooting transcription for this node.
 * @return Performance index for the event node.
 */
PerformanceIndex ComputePerformanceIndex(const EventTranscription& transcription);

/**
 * Compute the performance index from the transcription for the terminal node.
 * @param [in] transcription: multiple shooting transcription for this node.
 * @return Performance index for the terminal node.
 */
PerformanceIndex ComputePerformanceIndex(const TerminalTranscription& transcription);

/**
 * Compute only the performance index for a single intermediate node.
 * Corresponds to the performance index computed from the Transcription returned by "multiple_shooting::SetupIntermediateNode".
 * @param [in] optimalControlProblem: Definition of the optimal control problem
 * @param [in] discretizer: Integrator to use for creating the discrete dynamics.
 * @param [in] t: Start of the discrete interval
 * @param [in] dt: Duration of the interval
 * @param [in] x: State at start of the interval
 * @param [in] x_next: State at the end of the interval
 * @param [in] u: Input, taken to be constant across the interval.
 * @return Performance index for a single intermediate node.
 */
PerformanceIndex ComputeIntermediatePerformance(OptimalControlProblem& optimalControlProblem, DynamicsDiscretizer& discretizer, scalar_t t,
                                                scalar_t dt, const vector_t& x, const vector_t& x_next, const vector_t& u);

/**
 * Compute only the performance index for the event node.
 * Corresponds to the performance index computed from EventTranscription returned by "multiple_shooting::setEventNode".
 * @param [in] optimalControlProblem: Definition of the optimal control problem
 * @param [in] t: Time at the event node
 * @param [in] x: Pre-event state
 * @param [in] x_next: Post-event state
 * @return Performance index for the event node.
 */
PerformanceIndex ComputeEventPerformance(OptimalControlProblem& optimalControlProblem, scalar_t t, const vector_t& x,
                                         const vector_t& x_next);

/**
 * Compute only the performance index for the terminal node.
 * Corresponds to the performance index computed from TerminalTranscription returned by "multiple_shooting::setTerminalNode".
 * @param [in] optimalControlProblem: Definition of the optimal control problem
 * @param [in] t: Time at the terminal node
 * @param [in] x: Terminal state
 * @return Performance index for the terminal node.
 */
PerformanceIndex ComputeTerminalPerformance(OptimalControlProblem& optimalControlProblem, scalar_t t, const vector_t& x);

}  // namespace multiple_shooting
}  // namespace bolza
