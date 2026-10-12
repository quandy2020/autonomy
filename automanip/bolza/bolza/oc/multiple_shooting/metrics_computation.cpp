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

#include "bolza/oc/multiple_shooting/metrics_computation.hpp"

#include "bolza/oc/approximate_model/linear_quadratic_approximator.hpp"

namespace bolza {
namespace multiple_shooting {

Metrics ComputeMetrics(const Transcription& transcription) {
  const auto& constraintsSize = transcription.constraintsSize;

  Metrics metrics;

  // Cost
  metrics.cost = transcription.cost.f;

  // Dynamics
  metrics.dynamicsViolation = transcription.dynamics.f;

  // Equality constraints
  metrics.stateEqConstraint = ToConstraintArray(constraintsSize.stateEq, transcription.stateEqConstraints.f);
  metrics.stateInputEqConstraint = ToConstraintArray(constraintsSize.stateInputEq, transcription.stateInputEqConstraints.f);

  // Inequality constraints.
  metrics.stateIneqConstraint = ToConstraintArray(constraintsSize.stateIneq, transcription.stateIneqConstraints.f);
  metrics.stateInputIneqConstraint = ToConstraintArray(constraintsSize.stateInputIneq, transcription.stateInputIneqConstraints.f);

  return metrics;
}

Metrics ComputeMetrics(const EventTranscription& transcription) {
  const auto& constraintsSize = transcription.constraintsSize;

  Metrics metrics;

  // Cost
  metrics.cost = transcription.cost.f;

  // Dynamics
  metrics.dynamicsViolation = transcription.dynamics.f;

  // Equality constraints
  metrics.stateEqConstraint = ToConstraintArray(constraintsSize.stateEq, transcription.eqConstraints.f);

  // Inequality constraints.
  metrics.stateIneqConstraint = ToConstraintArray(constraintsSize.stateIneq, transcription.ineqConstraints.f);

  return metrics;
}

Metrics ComputeMetrics(const TerminalTranscription& transcription) {
  const auto& constraintsSize = transcription.constraintsSize;

  Metrics metrics;

  // Cost
  metrics.cost = transcription.cost.f;

  // Equality constraints
  metrics.stateEqConstraint = ToConstraintArray(constraintsSize.stateEq, transcription.eqConstraints.f);

  // Inequality constraints.
  metrics.stateIneqConstraint = ToConstraintArray(constraintsSize.stateIneq, transcription.ineqConstraints.f);

  return metrics;
}

Metrics ComputeIntermediateMetrics(OptimalControlProblem& optimalControlProblem, DynamicsDiscretizer& discretizer, scalar_t t, scalar_t dt,
                                   const vector_t& x, const vector_t& x_next, const vector_t& u) {
  // Dynamics
  auto dynamicsViolation = discretizer(*optimalControlProblem.dynamicsPtr, t, x, u, dt);
  dynamicsViolation -= x_next;

  // Precomputation
  constexpr auto request = Request::Cost + Request::SoftConstraint + Request::Constraint;
  optimalControlProblem.preComputationPtr->Request(request, t, x, u);

  // Compute metrics
  auto metrics = ComputeIntermediateMetrics(optimalControlProblem, t, x, u, std::move(dynamicsViolation));
  metrics.cost *= dt;  // consider dt

  return metrics;
}

Metrics ComputeTerminalMetrics(OptimalControlProblem& optimalControlProblem, scalar_t t, const vector_t& x) {
  // Precomputation
  constexpr auto request = Request::Cost + Request::SoftConstraint + Request::Constraint;
  optimalControlProblem.preComputationPtr->RequestFinal(request, t, x);

  return ComputeFinalMetrics(optimalControlProblem, t, x);
}

Metrics ComputeEventMetrics(OptimalControlProblem& optimalControlProblem, scalar_t t, const vector_t& x, const vector_t& x_next) {
  // Precomputation
  constexpr auto request = Request::Cost + Request::SoftConstraint + Request::Constraint + Request::Dynamics;
  optimalControlProblem.preComputationPtr->RequestPreJump(request, t, x);

  // Dynamics
  auto dynamicsViolation = optimalControlProblem.dynamicsPtr->ComputeJumpMap(t, x);
  dynamicsViolation -= x_next;

  return ComputePreJumpMetrics(optimalControlProblem, t, x, std::move(dynamicsViolation));
}

}  // namespace multiple_shooting
}  // namespace bolza
