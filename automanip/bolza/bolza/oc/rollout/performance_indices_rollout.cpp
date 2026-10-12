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

#include <bolza/core/integration/trapezoidal_integration.hpp>

#include <bolza/oc/rollout/performance_indices_rollout.hpp>

namespace bolza {
namespace PerformanceIndicesRollout {

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
scalar_t RolloutCost(cost_wraper_t costWraper, const scalar_array_t& timeTrajectory, const vector_array_t& stateTrajectory,
                     const vector_array_t& inputTrajectory) {
  const size_t N = timeTrajectory.size();
  assert(stateTrajectory.size() == N);
  assert(inputTrajectory.size() == N);

  scalar_array_t costTrajectory;
  costTrajectory.reserve(N);
  for (size_t i = 0; i < N; i++) {
    costTrajectory.push_back(costWraper(timeTrajectory[i], stateTrajectory[i], inputTrajectory[i]));
  }

  return TrapezoidalIntegration(timeTrajectory, costTrajectory, 0.0);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
scalar_t RolloutConstraint(constraints_wraper_t constraintWraper, const scalar_array_t& timeTrajectory,
                           const vector_array_t& stateTrajectory, const vector_array_t& inputTrajectory) {
  const size_t N = timeTrajectory.size();
  assert(stateTrajectory.size() == N);
  assert(inputTrajectory.size() == N);

  scalar_array_t constraintTrajectoryISE;
  constraintTrajectoryISE.reserve(N);
  for (size_t i = 0; i < N; i++) {
    const auto constraint = constraintWraper(timeTrajectory[i], stateTrajectory[i], inputTrajectory[i]);
    constraintTrajectoryISE.push_back(constraint.squaredNorm());
  }

  return TrapezoidalIntegration(timeTrajectory, constraintTrajectoryISE, 0.0);
}

}  // namespace PerformanceIndicesRollout
}  // namespace bolza
