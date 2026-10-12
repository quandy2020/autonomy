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
 * @file ipm_performance_index_computation.hpp
 * @brief Compute the performance index from the transcription for a single intermediate node.
 */


#include <bolza/core/types.hpp>
#include <bolza/core/model_data/metrics.hpp>
#include <bolza/oc/multiple_shooting/transcription.hpp>
#include <bolza/oc/oc_data/performance_index.hpp>

namespace bolza {
namespace ipm {

/**
 * Compute the performance index from the transcription for a single intermediate node.
 *
 * @param [in] transcription: transcription computed by "multiple_shooting::SetupIntermediateNode"
 * @param [in] dt: Duration of the interval
 * @param [in] barrierParam: Barrier parameter of the interior point method
 * @param [in] slackStateIneq: Slack variable of the state inequality constraints
 * @param [in] slackStateInputIneq: Slack variable of the state-input inequality constraints
 * @return Performance index for a single intermediate node
 */
PerformanceIndex ComputePerformanceIndex(const multiple_shooting::Transcription& transcription, scalar_t dt, scalar_t barrierParam,
                                         const vector_t& slackStateIneq, const vector_t& slackStateInputIneq);

/**
 * Compute the performance index from the transcription for the terminal node.
 *
 * @param [in] transcription: transcription computed by "multiple_shooting::SetupTerminalNode"
 * @param [in] barrierParam: Barrier parameter of the interior point method
 * @param [in] slackIneq: Slack variable of the inequality constraints
 * @return Performance index for the terminal node
 */
PerformanceIndex ComputePerformanceIndex(const multiple_shooting::TerminalTranscription& transcription, scalar_t barrierParam,
                                         const vector_t& slackIneq);

/**
 * Compute the performance index from the transcription for the event node.
 *
 * @param [in] transcription: transcription computed by "multiple_shooting::SetupEventNode"
 * @param [in] barrierParam: Barrier parameter of the interior point method
 * @param [in] slackIneq: Slack variable of the inequality constraints
 * @return Performance index for the event node
 */
PerformanceIndex ComputePerformanceIndex(const multiple_shooting::EventTranscription& transcription, scalar_t barrierParam,
                                         const vector_t& slackIneq);

/**
 * @brief Compute only the performance index for a single intermediate node.
 *
 * Corresponds to the performance index computed from the multiple_shooting::Transcription returned by
 *
 * "multiple_shooting::SetupIntermediateNode"
 * @param [in] optimalControlProblem: Definition of the optimal control problem
 * @param [in] discretizer: Integrator to use for creating the discrete dynamics
 * @param [in] t: Start of the discrete interval
 * @param [in] dt: Duration of the interval
 * @param [in] x: State at start of the interval
 * @param [in] x_next: State at the end of the interval
 * @param [in] u: Input, taken to be constant across the interval
 * @param [in] barrierParam: Barrier parameter of the interior point method
 * @param [in] slackStateIneq: Slack variable of the state inequality constraints
 * @param [in] slackStateInputIneq: Slack variable of the state-input inequality constraints
 * @param [in] enableStateInequalityConstraints: Enable State Inequality Constraints.
 * @return Performance index for a single intermediate node
 */
PerformanceIndex ComputeIntermediatePerformance(OptimalControlProblem& optimalControlProblem, DynamicsDiscretizer& discretizer, scalar_t t,
                                                scalar_t dt, const vector_t& x, const vector_t& x_next, const vector_t& u,
                                                scalar_t barrierParam, const vector_t& slackStateIneq, const vector_t& slackStateInputIneq,
                                                bool enableStateInequalityConstraints);

/**
 * Compute only the performance index for the terminal node.
 * Corresponds to the performance index computed from multiple_shooting::TerminalTranscription returned by
 * "multiple_shooting::setTerminalNode"
 *
 * @param [in] optimalControlProblem: Definition of the optimal control problem
 * @param [in] t: Time at the terminal node
 * @param [in] x: Terminal state
 * @param [in] barrierParam: Barrier parameter of the interior point method
 * @param [in] slackIneq: Slack variable of the inequality constraints
 * @return Performance index for the terminal node
 */
PerformanceIndex ComputeTerminalPerformance(OptimalControlProblem& optimalControlProblem, scalar_t t, const vector_t& x,
                                            scalar_t barrierParam, const vector_t& slackIneq);

/**
 * Compute only the performance index for the event node.
 * Corresponds to the performance index computed from multiple_shooting::EventTranscription returned by
 * "multiple_shooting::setEventNode"
 *
 * @param [in] optimalControlProblem: Definition of the optimal control problem
 * @param [in] t: Time at the event node
 * @param [in] x: Pre-event state
 * @param [in] x_next: Post-event state
 * @param [in] barrierParam: Barrier parameter of the interior point method
 * @param [in] slackIneq: Slack variable of the inequality constraints
 * @return Performance index for the event node
 */
PerformanceIndex ComputeEventPerformance(OptimalControlProblem& optimalControlProblem, scalar_t t, const vector_t& x,
                                         const vector_t& x_next, scalar_t barrierParam, const vector_t& slackIneq);

/**
 * Computes the PerformanceIndex of the interior point method based on a given Metrics at an intermediate node.
 *
 * @param [in] metrics: Metrics at an intermediate node
 * @param [in] dt: Duration of the interval
 * @param [in] barrierParam: Barrier parameter of the interior point method
 * @param [in] slackStateIneq: Slack variable of the state inequality constraints
 * @param [in] slackStateInputIneq: Slack variable of the state-input inequality constraints
 * @return Performance index of the interior point method
 */
PerformanceIndex ToPerformanceIndex(const Metrics& metrics, scalar_t dt, scalar_t barrierParam, const vector_t& slackStateIneq,
                                    const vector_t& slackStateInputIneq);

/**
 * Computes the PerformanceIndex of the interior point method based on a given Metrics at the event or terminal node.
 *
 * @param [in] metrics: Metrics at the event or terminal node
 * @param [in] barrierParam: Barrier parameter of the interior point method
 * @param [in] slackIneq: Slack variable of the inequality constraints
 * @return Performance index of the interior point method
 */
PerformanceIndex ToPerformanceIndex(const Metrics& metrics, scalar_t barrierParam, const vector_t& slackIneq);

}  // namespace ipm
}  // namespace bolza
