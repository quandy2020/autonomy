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

#include "bolza/mpcnet/mpcnet_interface_base.hpp"

namespace bolza {
namespace mpcnet {

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void MPCNetInterfaceBase::StartDataGeneration(scalar_t alpha, const std::string& policyFilePath, scalar_t timeStep, size_t dataDecimation,
                                              size_t nSamples, const matrix_t& samplingCovariance,
                                              const std::vector<SystemObservation>& initialObservations,
                                              const std::vector<ModeSchedule>& modeSchedules,
                                              const std::vector<TargetTrajectories>& targetTrajectories) {
  mpcnetRolloutManagerPtr_->StartDataGeneration(alpha, policyFilePath, timeStep, dataDecimation, nSamples, samplingCovariance,
                                                initialObservations, modeSchedules, targetTrajectories);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
bool MPCNetInterfaceBase::IsDataGenerationDone() {
  return mpcnetRolloutManagerPtr_->IsDataGenerationDone();
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
data_array_t MPCNetInterfaceBase::GetGeneratedData() {
  return mpcnetRolloutManagerPtr_->GetGeneratedData();
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void MPCNetInterfaceBase::StartPolicyEvaluation(scalar_t alpha, const std::string& policyFilePath, scalar_t timeStep,
                                                const std::vector<SystemObservation>& initialObservations,
                                                const std::vector<ModeSchedule>& modeSchedules,
                                                const std::vector<TargetTrajectories>& targetTrajectories) {
  mpcnetRolloutManagerPtr_->StartPolicyEvaluation(alpha, policyFilePath, timeStep, initialObservations, modeSchedules, targetTrajectories);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
bool MPCNetInterfaceBase::IsPolicyEvaluationDone() {
  return mpcnetRolloutManagerPtr_->IsPolicyEvaluationDone();
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
metrics_array_t MPCNetInterfaceBase::GetComputedMetrics() {
  return mpcnetRolloutManagerPtr_->GetComputedMetrics();
}

}  // namespace mpcnet
}  // namespace bolza
