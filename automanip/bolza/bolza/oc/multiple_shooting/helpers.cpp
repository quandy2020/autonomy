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

#include "bolza/oc/multiple_shooting/helpers.hpp"

#include <bolza/core/control/feedforward_controller.hpp>
#include <bolza/core/control/linear_controller.hpp>

namespace bolza {
namespace multiple_shooting {

void RemapProjectedInput(const std::vector<VectorFunctionLinearApproximation>& constraintsProjection, const vector_array_t& deltaXSol,
                         vector_array_t& deltaUSol) {
  vector_t tmp;  // 1 temporary for re-use.
  for (int i = 0; i < deltaUSol.size(); ++i) {
    if (constraintsProjection[i].f.size() > 0) {
      tmp.noalias() = constraintsProjection[i].dfdu * deltaUSol[i];
      deltaUSol[i] = tmp + constraintsProjection[i].f;
      deltaUSol[i].noalias() += constraintsProjection[i].dfdx * deltaXSol[i];
    }
  }
}

void RemapProjectedGain(const std::vector<VectorFunctionLinearApproximation>& constraintsProjection, matrix_array_t& KMatrices) {
  matrix_t tmp;  // 1 temporary for re-use.
  for (int i = 0; i < KMatrices.size(); ++i) {
    if (constraintsProjection[i].f.size() > 0) {
      tmp.noalias() = constraintsProjection[i].dfdu * KMatrices[i];
      KMatrices[i] = tmp + constraintsProjection[i].dfdx;
    }
  }
}

PrimalSolution ToPrimalSolution(const std::vector<AnnotatedTime>& time, ModeSchedule&& modeSchedule, vector_array_t&& x,
                                vector_array_t&& u) {
  // Correct for missing inputs at PreEvents and the terminal time
  for (int i = 0; i < u.size(); ++i) {
    if (time[i].event == AnnotatedTime::Event::PreEvent && i > 0) {
      u[i] = u[i - 1];
    }
  }
  // Repeat last input to make equal length vectors
  u.push_back(u.back());

  // Construct nominal time, state and input trajectories
  PrimalSolution primalSolution;
  primalSolution.timeTrajectory_ = ToTime(time);
  primalSolution.postEventIndices_ = ToPostEventIndices(time);
  primalSolution.stateTrajectory_ = std::move(x);
  primalSolution.inputTrajectory_ = std::move(u);
  primalSolution.modeSchedule_ = std::move(modeSchedule);
  primalSolution.controllerPtr_.reset(new FeedforwardController(primalSolution.timeTrajectory_, primalSolution.inputTrajectory_));
  return primalSolution;
}

PrimalSolution ToPrimalSolution(const std::vector<AnnotatedTime>& time, ModeSchedule&& modeSchedule, vector_array_t&& x, vector_array_t&& u,
                                matrix_array_t&& KMatrices) {
  // Compute feedback, before x and u are moved to primal solution
  // see doc/LQR_full.pdf for detailed derivation for feedback terms
  vector_array_t uff = u;  // Copy and adapt in loop
  for (int i = 0; i < KMatrices.size(); ++i) {
    if (time[i].event == AnnotatedTime::Event::PreEvent && i > 0) {
      uff[i] = uff[i - 1];
      KMatrices[i] = KMatrices[i - 1];
    } else {
      // Linear controller has convention u = uff + K * x;
      // We computed u = u'(t) + K (x - x'(t));
      // >> uff = u'(t) - K x'(t)
      uff[i].noalias() -= KMatrices[i] * x[i];
    }
  }
  // Copy last one to get correct length
  uff.push_back(uff.back());
  KMatrices.push_back(KMatrices.back());

  // Correct for missing inputs at PreEvents and the terminal time
  for (int i = 0; i < u.size(); ++i) {
    if (time[i].event == AnnotatedTime::Event::PreEvent && i > 0) {
      u[i] = u[i - 1];
    }
  }
  // Repeat last input to make equal length vectors
  u.push_back(u.back());

  // Construct nominal time, state and input trajectories
  PrimalSolution primalSolution;
  primalSolution.timeTrajectory_ = ToTime(time);
  primalSolution.postEventIndices_ = ToPostEventIndices(time);
  primalSolution.stateTrajectory_ = std::move(x);
  primalSolution.inputTrajectory_ = std::move(u);
  primalSolution.modeSchedule_ = std::move(modeSchedule);
  primalSolution.controllerPtr_.reset(new LinearController(primalSolution.timeTrajectory_, std::move(uff), std::move(KMatrices)));
  return primalSolution;
}

ProblemMetrics ToProblemMetrics(const std::vector<AnnotatedTime>& time, std::vector<Metrics>&& metrics) {
  assert(time.size() > 1);
  assert(metrics.size() == time.size());

  // Problem horizon
  const int N = static_cast<int>(time.size()) - 1;

  // resize
  ProblemMetrics problemMetrics;
  problemMetrics.intermediates.reserve(N);
  problemMetrics.preJumps.reserve(N / 10);  // the size is just a guess
  problemMetrics.final = std::move(metrics.back());

  for (int i = 0; i < N; ++i) {
    if (time[i].event == AnnotatedTime::Event::PreEvent) {
      problemMetrics.preJumps.push_back(std::move(metrics[i]));
    } else {
      problemMetrics.intermediates.push_back(std::move(metrics[i]));
    }
  }

  return problemMetrics;
}

}  // namespace multiple_shooting
}  // namespace bolza
