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

#include "bolza/oc/multiple_shooting/performance_index_computation.hpp"

#include <bolza/core/model_data/metrics.hpp>

#include "bolza/oc/approximate_model/linear_quadratic_approximator.hpp"
#include "bolza/oc/multiple_shooting/metrics_computation.hpp"

namespace bolza {
namespace multiple_shooting {

PerformanceIndex ComputePerformanceIndex(const Transcription& transcription, scalar_t dt) {
  PerformanceIndex performance;

  // Dynamics
  performance.dynamicsViolationSSE = dt * transcription.dynamics.f.squaredNorm();

  // Costs
  performance.cost = transcription.cost.f;

  // State-input equality constraints
  performance.equalityConstraintsSSE =
      dt * (GetEqConstraintsSSE(transcription.stateEqConstraints.f) + GetEqConstraintsSSE(transcription.stateInputEqConstraints.f));

  // Inequality constraints.
  performance.inequalityConstraintsSSE =
      dt * (GetIneqConstraintsSSE(transcription.stateIneqConstraints.f) + GetIneqConstraintsSSE(transcription.stateInputIneqConstraints.f));

  return performance;
}

PerformanceIndex ComputePerformanceIndex(const EventTranscription& transcription) {
  PerformanceIndex performance;

  // Dynamics
  performance.dynamicsViolationSSE = transcription.dynamics.f.squaredNorm();

  performance.cost = transcription.cost.f;

  // Equality constraints
  performance.equalityConstraintsSSE = GetEqConstraintsSSE(transcription.eqConstraints.f);

  // State inequality constraints.
  performance.inequalityConstraintsSSE = GetIneqConstraintsSSE(transcription.ineqConstraints.f);

  return performance;
}

PerformanceIndex ComputePerformanceIndex(const TerminalTranscription& transcription) {
  PerformanceIndex performance;

  performance.cost = transcription.cost.f;

  // Equality constraints
  performance.equalityConstraintsSSE = GetEqConstraintsSSE(transcription.eqConstraints.f);

  // State inequality constraints.
  performance.inequalityConstraintsSSE = GetIneqConstraintsSSE(transcription.ineqConstraints.f);

  return performance;
}

PerformanceIndex ComputeIntermediatePerformance(OptimalControlProblem& optimalControlProblem, DynamicsDiscretizer& discretizer, scalar_t t,
                                                scalar_t dt, const vector_t& x, const vector_t& x_next, const vector_t& u) {
  const auto metrics = ComputeIntermediateMetrics(optimalControlProblem, discretizer, t, dt, x, x_next, u);
  return ToPerformanceIndex(metrics, dt);
}

PerformanceIndex ComputeEventPerformance(OptimalControlProblem& optimalControlProblem, scalar_t t, const vector_t& x,
                                         const vector_t& x_next) {
  const auto metrics = ComputeEventMetrics(optimalControlProblem, t, x, x_next);
  return ToPerformanceIndex(metrics);
}

PerformanceIndex ComputeTerminalPerformance(OptimalControlProblem& optimalControlProblem, scalar_t t, const vector_t& x) {
  const auto metrics = ComputeTerminalMetrics(optimalControlProblem, t, x);
  return ToPerformanceIndex(metrics);
}

}  // namespace multiple_shooting
}  // namespace bolza
