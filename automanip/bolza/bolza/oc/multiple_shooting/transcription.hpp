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
 * @file transcription.hpp
 * @brief The size of each constraint terms for each type.
 */


#include <bolza/core/types.hpp>
#include <bolza/core/integration/sensitivity_integrator.hpp>

#include "bolza/oc/multiple_shooting/projection_multiplier_coefficients.hpp"
#include "bolza/oc/oc_problem/optimal_control_problem.hpp"

namespace bolza {
namespace multiple_shooting {

/**
 * The size of each constraint terms for each type.
 */
struct ConstraintsSize {
  size_array_t stateEq;
  size_array_t stateInputEq;
  size_array_t stateIneq;
  size_array_t stateInputIneq;
};

/**
 * Results of the transcription at an intermediate node
 */
struct Transcription {
  ConstraintsSize constraintsSize;
  ScalarFunctionQuadraticApproximation cost;
  VectorFunctionLinearApproximation dynamics;
  VectorFunctionLinearApproximation stateEqConstraints;
  VectorFunctionLinearApproximation stateInputEqConstraints;
  VectorFunctionLinearApproximation stateIneqConstraints;
  VectorFunctionLinearApproximation stateInputIneqConstraints;
  VectorFunctionLinearApproximation constraintsProjection;
  ProjectionMultiplierCoefficients projectionMultiplierCoefficients;
};

/**
 * Compute the multiple shooting transcription for a single intermediate node.
 *
 * @param [in] optimalControlProblem: Definition of the optimal control problem
 * @param [in] sensitivityDiscretizer: Integrator to use for creating the discrete dynamics.
 * @param [in] t: Start of the discrete interval
 * @param [in] dt: Duration of the interval
 * @param [in] x: State at start of the interval
 * @param [in] x_next: State at the end of the interval
 * @param [in] u: Input, taken to be constant across the interval.
 * @return multiple shooting transcription for this node.
 */
Transcription SetupIntermediateNode(OptimalControlProblem& optimalControlProblem, DynamicsSensitivityDiscretizer& sensitivityDiscretizer,
                                    scalar_t t, scalar_t dt, const vector_t& x, const vector_t& x_next, const vector_t& u);

/**
 * Apply the state-input equality constraint projection for a single intermediate node transcription.
 *
 * @param [in] transcription: Transcription for a single intermediate node
 * @param [in] extractProjectionMultiplier: Whether to extract the projection multiplier.
 */
void ProjectTranscription(Transcription& transcription, bool extractProjectionMultiplier = false);

/**
 * Results of the transcription at a terminal node
 */
struct TerminalTranscription {
  ConstraintsSize constraintsSize;
  ScalarFunctionQuadraticApproximation cost;
  VectorFunctionLinearApproximation eqConstraints;
  VectorFunctionLinearApproximation ineqConstraints;
};

/**
 * Compute the multiple shooting transcription the terminal node.
 *
 * @param [in] optimalControlProblem: Definition of the optimal control problem
 * @param [in] t: Time at the terminal node
 * @param [in] x: Terminal state
 * @return multiple shooting transcription for the terminal node.
 */
TerminalTranscription SetupTerminalNode(OptimalControlProblem& optimalControlProblem, scalar_t t, const vector_t& x);

/**
 * Results of the transcription at an event
 */
struct EventTranscription {
  ConstraintsSize constraintsSize;
  ScalarFunctionQuadraticApproximation cost;
  VectorFunctionLinearApproximation dynamics;
  VectorFunctionLinearApproximation eqConstraints;
  VectorFunctionLinearApproximation ineqConstraints;
};

/**
 * Compute the jump transcription at an event node.
 *
 * @param [in] optimalControlProblem: Definition of the optimal control problem
 * @param [in] t: Time at the event node
 * @param [in] x: Pre-event state
 * @param [in] x_next: Post-event state
 * @return multiple shooting transcription for the event node.
 */
EventTranscription SetupEventNode(OptimalControlProblem& optimalControlProblem, scalar_t t, const vector_t& x, const vector_t& x_next);

}  // namespace multiple_shooting
}  // namespace bolza
