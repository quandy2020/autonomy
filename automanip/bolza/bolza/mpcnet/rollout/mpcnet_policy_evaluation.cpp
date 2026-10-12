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

#include "bolza/mpcnet/rollout/mpcnet_policy_evaluation.hpp"

namespace bolza {
namespace mpcnet {

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
metrics_t MPCNetPolicyEvaluation::Run(scalar_t alpha, const std::string& policyFilePath, scalar_t timeStep,
                                      const SystemObservation& initialObservation, const ModeSchedule& modeSchedule,
                                      const TargetTrajectories& targetTrajectories) {
  // declare metrics
  metrics_t metrics;

  // set system
  set(alpha, policyFilePath, initialObservation, modeSchedule, targetTrajectories);

  // run policy evaluation
  try {
    while (systemObservation_.time <= targetTrajectories.timeTrajectory.back()) {
      // step system
      Step(timeStep);

      // incurred quantities
      const scalar_t time = primalSolution_.timeTrajectory_.front();
      const vector_t state = primalSolution_.stateTrajectory_.front();
      const vector_t input = behavioralControllerPtr_->ComputeInput(time, state);
      metrics.incurredHamiltonian += mpcPtr_->GetSolverPtr()->GetHamiltonian(time, state, input).f * timeStep;
    }
  } catch (const std::exception& e) {
    // print error for exceptions
    std::cerr << "[MPCNetPolicyEvaluation::Run] a standard exception was caught, with message: " << e.what() << "\n";
    // this policy evaluation run failed, incurred quantities are not reported
    metrics.incurredHamiltonian = std::numeric_limits<scalar_t>::quiet_NaN();
  }

  // report survival time
  metrics.survivalTime = systemObservation_.time;

  // return metrics
  return metrics;
}

}  // namespace mpcnet
}  // namespace bolza
