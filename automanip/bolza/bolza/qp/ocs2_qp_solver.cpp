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

#include "bolza/qp/ocs2_qp_solver.hpp"

#include "bolza/qp/qp_discrete_transcription.hpp"
#include "bolza/qp/qp_solver.hpp"

namespace bolza {
namespace qp_solver {

ContinuousTrajectory SolveLinearQuadraticOptimalControlProblem(OptimalControlProblem& optimalControProblem,
                                                               const ContinuousTrajectory& nominalTrajectory,
                                                               const vector_t& initialState) {
  // Approximate
  const auto lqApproximation = GetLinearQuadraticApproximation(optimalControProblem, nominalTrajectory);

  // Solve for an update step
  ContinuousTrajectory deltaSolution;
  deltaSolution.timeTrajectory = nominalTrajectory.timeTrajectory;
  std::tie(deltaSolution.stateTrajectory, deltaSolution.inputTrajectory) =
      SolveLinearQuadraticProblem(lqApproximation, initialState - nominalTrajectory.stateTrajectory.front());

  // Take a full step: Add update to nominal trajectory
  return nominalTrajectory + deltaSolution;
}

}  // namespace qp_solver
}  // namespace bolza
