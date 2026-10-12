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

#include "bolza/ipm/ipm_performance_index_computation.hpp"

#include <bolza/core/model_data/metrics.hpp>

#include <bolza/oc/approximate_model/linear_quadratic_approximator.hpp>
#include <bolza/oc/multiple_shooting/metrics_computation.hpp>
#include <bolza/oc/multiple_shooting/performance_index_computation.hpp>

namespace bolza {
namespace ipm {

PerformanceIndex ComputePerformanceIndex(const multiple_shooting::Transcription& transcription, scalar_t dt, scalar_t barrierParam,
                                         const vector_t& slackStateIneq, const vector_t& slackStateInputIneq) {
  auto performance = multiple_shooting::ComputePerformanceIndex(transcription, dt);

  if (slackStateIneq.size() > 0) {
    performance.cost -= dt * barrierParam * slackStateIneq.array().log().sum();
  }
  if (slackStateInputIneq.size() > 0) {
    performance.cost -= dt * barrierParam * slackStateInputIneq.array().log().sum();
  }

  if (transcription.stateIneqConstraints.f.size() > 0) {
    performance.equalityConstraintsSSE += dt * (transcription.stateIneqConstraints.f - slackStateIneq).squaredNorm();
  }
  if (transcription.stateInputIneqConstraints.f.size() > 0) {
    performance.equalityConstraintsSSE += dt * (transcription.stateInputIneqConstraints.f - slackStateInputIneq).squaredNorm();
  }

  return performance;
}

PerformanceIndex ComputePerformanceIndex(const multiple_shooting::TerminalTranscription& transcription, scalar_t barrierParam,
                                         const vector_t& slackIneq) {
  auto performance = multiple_shooting::ComputePerformanceIndex(transcription);

  if (slackIneq.size() > 0) {
    performance.cost -= barrierParam * slackIneq.array().log().sum();
  }

  if (transcription.ineqConstraints.f.size() > 0) {
    performance.equalityConstraintsSSE += (transcription.ineqConstraints.f - slackIneq).squaredNorm();
  }

  return performance;
}

PerformanceIndex ComputePerformanceIndex(const multiple_shooting::EventTranscription& transcription, scalar_t barrierParam,
                                         const vector_t& slackIneq) {
  auto performance = multiple_shooting::ComputePerformanceIndex(transcription);

  if (slackIneq.size() > 0) {
    performance.cost -= barrierParam * slackIneq.array().log().sum();
  }

  if (transcription.ineqConstraints.f.size() > 0) {
    performance.equalityConstraintsSSE += (transcription.ineqConstraints.f - slackIneq).squaredNorm();
  }

  return performance;
}

PerformanceIndex ComputeIntermediatePerformance(OptimalControlProblem& optimalControlProblem, DynamicsDiscretizer& discretizer, scalar_t t,
                                                scalar_t dt, const vector_t& x, const vector_t& x_next, const vector_t& u,
                                                scalar_t barrierParam, const vector_t& slackStateIneq, const vector_t& slackStateInputIneq,
                                                bool enableStateInequalityConstraints) {
  auto metrics = multiple_shooting::ComputeIntermediateMetrics(optimalControlProblem, discretizer, t, dt, x, x_next, u);
  if (!enableStateInequalityConstraints) {
    metrics.stateIneqConstraint.clear();
  }
  return ToPerformanceIndex(metrics, dt, barrierParam, slackStateIneq, slackStateInputIneq);
}

PerformanceIndex ComputeEventPerformance(OptimalControlProblem& optimalControlProblem, scalar_t t, const vector_t& x,
                                         const vector_t& x_next, scalar_t barrierParam, const vector_t& slackIneq) {
  const auto metrics = multiple_shooting::ComputeEventMetrics(optimalControlProblem, t, x, x_next);
  return ToPerformanceIndex(metrics, barrierParam, slackIneq);
}

PerformanceIndex ComputeTerminalPerformance(OptimalControlProblem& optimalControlProblem, scalar_t t, const vector_t& x,
                                            scalar_t barrierParam, const vector_t& slackIneq) {
  const auto metrics = multiple_shooting::ComputeTerminalMetrics(optimalControlProblem, t, x);
  return ToPerformanceIndex(metrics, barrierParam, slackIneq);
}

PerformanceIndex ToPerformanceIndex(const Metrics& metrics, scalar_t dt, scalar_t barrierParam, const vector_t& slackStateIneq,
                                    const vector_t& slackStateInputIneq) {
  PerformanceIndex performance = ToPerformanceIndex(metrics, dt);

  if (slackStateIneq.size() > 0) {
    performance.cost -= dt * barrierParam * slackStateIneq.array().log().sum();
    performance.equalityConstraintsSSE += dt * (ToVector(metrics.stateIneqConstraint) - slackStateIneq).squaredNorm();
  }

  if (slackStateInputIneq.size() > 0) {
    performance.cost -= dt * barrierParam * slackStateInputIneq.array().log().sum();
    performance.equalityConstraintsSSE += dt * (ToVector(metrics.stateInputIneqConstraint) - slackStateInputIneq).squaredNorm();
  }

  return performance;
}

PerformanceIndex ToPerformanceIndex(const Metrics& metrics, scalar_t barrierParam, const vector_t& slackIneq) {
  PerformanceIndex performance = ToPerformanceIndex(metrics);

  if (slackIneq.size() > 0) {
    performance.cost -= barrierParam * slackIneq.array().log().sum();
    performance.equalityConstraintsSSE += (ToVector(metrics.stateIneqConstraint) - slackIneq).squaredNorm();
  }

  return performance;
}

}  // namespace ipm
}  // namespace bolza
