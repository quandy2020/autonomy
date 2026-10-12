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

#include "bolza/ddp/slq.hpp"

#include "bolza/ddp/ddp_helper_functions.hpp"
#include "bolza/ddp/riccati_equations/riccati_modification_interpolation.hpp"

namespace bolza {

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
SLQ::SLQ(ddp::Settings ddpSettings, const RolloutBase& rollout, const OptimalControlProblem& optimalControlProblem,
         const Initializer& initializer)
    : GaussNewtonDDP(std::move(ddpSettings), rollout, optimalControlProblem, initializer) {
  if (Settings().algorithm_ != ddp::Algorithm::SLQ) {
    throw std::runtime_error("[SLQ] In DDP setting the algorithm name is set \"" + ddp::ToAlgorithmName(Settings().algorithm_) +
                             "\" while SLQ is instantiated!");
  }

  allSsTrajectoryStock_.resize(Settings().nThreads_);
  SsNormalizedTimeTrajectoryStock_.resize(Settings().nThreads_);
  SsNormalizedEventsPastTheEndIndecesStock_.resize(Settings().nThreads_);
  // Riccati Solver
  riccatiEquationsPtrStock_.clear();
  riccatiEquationsPtrStock_.reserve(Settings().nThreads_);
  riccatiIntegratorPtrStock_.clear();
  riccatiIntegratorPtrStock_.reserve(Settings().nThreads_);

  const auto integratorType = Settings().backwardPassIntegratorType_;
  if (integratorType != IntegratorType::ODE45 && integratorType != IntegratorType::BULIRSCH_STOER &&
      integratorType != IntegratorType::ODE45_OCS2 && integratorType != IntegratorType::RK4) {
    throw(std::runtime_error("Unsupported Riccati equation integrator type: " +
                             integrator_type::ToString(Settings().backwardPassIntegratorType_)));
  }

  for (size_t i = 0; i < Settings().nThreads_; i++) {
    bool preComputeRiccatiTerms = Settings().preComputeRiccatiTerms_ && (Settings().strategy_ == search_strategy::Type::LINE_SEARCH);
    bool isRiskSensitive = !numerics::almost_eq(Settings().riskSensitiveCoeff_, 0.0);
    riccatiEquationsPtrStock_.emplace_back(new ContinuousTimeRiccatiEquations(preComputeRiccatiTerms, isRiskSensitive));
    riccatiEquationsPtrStock_.back()->setRiskSensitiveCoefficient(Settings().riskSensitiveCoeff_);
    riccatiIntegratorPtrStock_.emplace_back(NewIntegrator(integratorType));
  }  // end of i loop

  Eigen::initParallel();
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void SLQ::ApproximateIntermediateLQ(const DualSolution& dualSolution, PrimalDataContainer& primalData) {
  // create alias
  const auto& timeTrajectory = primalData.primalSolution.timeTrajectory_;
  const auto& stateTrajectory = primalData.primalSolution.stateTrajectory_;
  const auto& inputTrajectory = primalData.primalSolution.inputTrajectory_;
  const auto& postEventIndices = primalData.primalSolution.postEventIndices_;
  const auto& multiplierTrajectory = dualSolution.intermediates;
  auto& modelDataTrajectory = primalData.modelDataTrajectory;

  modelDataTrajectory.clear();
  modelDataTrajectory.resize(timeTrajectory.size());

  nextTimeIndex_ = 0;
  nextTaskId_ = 0;
  auto task = [&]() {
    const size_t taskId = nextTaskId_++;  // assign task ID (atomic)

    // get next time index is atomic
    size_t timeIndex;
    while ((timeIndex = nextTimeIndex_++) < timeTrajectory.size()) {
      // approximate LQ for the given time index
      bolza::ApproximateIntermediateLQ(optimalControlProblemStock_[taskId], timeTrajectory[timeIndex], stateTrajectory[timeIndex],
                                      inputTrajectory[timeIndex], multiplierTrajectory[timeIndex], modelDataTrajectory[timeIndex]);

      // checking the numerical properties
      if (Settings().checkNumericalStability_) {
        const auto errSize =
            CheckSize(modelDataTrajectory[timeIndex], stateTrajectory[timeIndex].rows(), inputTrajectory[timeIndex].rows());
        if (!errSize.empty()) {
          throw std::runtime_error("[SLQ::ApproximateIntermediateLQ] Mismatch in dimensions at intermediate time: " +
                                   std::to_string(timeTrajectory[timeIndex]) + "\n" + errSize);
        }
        const std::string errProperties = CheckDynamicsProperties(modelDataTrajectory[timeIndex]) +
                                          CheckCostProperties(modelDataTrajectory[timeIndex]) +
                                          CheckConstraintProperties(modelDataTrajectory[timeIndex]);
        if (!errProperties.empty()) {
          throw std::runtime_error("[SLQ::ApproximateIntermediateLQ] Ill-posed problem at intermediate time: " +
                                   std::to_string(timeTrajectory[timeIndex]) + "\n" + errProperties);
        }
      }
    }  // end of while loop
  };

  RunParallel(task, Settings().nThreads_);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void SLQ::CalculateControllerWorker(size_t timeIndex, const PrimalDataContainer& primalData, const DualDataContainer& dualData,
                                    LinearController& dstController) {
  const auto time = primalData.primalSolution.timeTrajectory_[timeIndex];

  const vector_t& nominalState = primalData.primalSolution.stateTrajectory_[timeIndex];
  const vector_t& nominalInput = primalData.primalSolution.inputTrajectory_[timeIndex];

  // BmProjected
  const matrix_t& projectedBm = dualData.projectedModelDataTrajectory[timeIndex].dynamics.dfdu;

  // PmProjected
  const matrix_t& projectedPm = dualData.projectedModelDataTrajectory[timeIndex].cost.dfdux;
  // RvProjected
  const vector_t& projectedRv = dualData.projectedModelDataTrajectory[timeIndex].cost.dfdu;
  // EvProjected
  const vector_t& EvProjected = dualData.projectedModelDataTrajectory[timeIndex].stateInputEqConstraint.f;
  // CmProjected
  const matrix_t& CmProjected = dualData.projectedModelDataTrajectory[timeIndex].stateInputEqConstraint.dfdx;
  // projector
  const matrix_t& Qu = dualData.riccatiModificationTrajectory[timeIndex].constraintNullProjector_;
  // deltaGm, projected feedback
  matrix_t projectedKm = dualData.riccatiModificationTrajectory[timeIndex].deltaGm_;
  // deltaGv, projected feedforward
  vector_t projectedLv = dualData.riccatiModificationTrajectory[timeIndex].deltaGv_;

  // projectedKm = projectedPm + projectedBm^t * Sm
  projectedKm = -(projectedKm + projectedPm);
  projectedKm.noalias() -= projectedBm.transpose() * dualData.valueFunctionTrajectory[timeIndex].dfdxx;

  // projectedLv = projectedRv + projectedBm^t * Sv
  projectedLv = -(projectedLv + projectedRv);
  projectedLv.noalias() -= projectedBm.transpose() * dualData.valueFunctionTrajectory[timeIndex].dfdx;

  // feedback gains
  dstController.gainArray_[timeIndex] = -CmProjected;
  dstController.gainArray_[timeIndex].noalias() += Qu * projectedKm;

  // bias input
  dstController.biasArray_[timeIndex] = nominalInput;
  dstController.biasArray_[timeIndex].noalias() -= dstController.gainArray_[timeIndex] * nominalState;
  dstController.deltaBiasArray_[timeIndex] = -EvProjected;
  dstController.deltaBiasArray_[timeIndex].noalias() += Qu * projectedLv;
}

/******************************************************************************************************/
/******************************************************************************************************/
/***************************************************************************************************** */
scalar_t SLQ::SolveSequentialRiccatiEquations(const ScalarFunctionQuadraticApproximation& finalValueFunction) {
  // fully compute the riccatiModifications and projected modelData
  // number of the intermediate LQ variables
  const size_t N = nominalPrimalData_.primalSolution.timeTrajectory_.size();

  nominalDualData_.riccatiModificationTrajectory.resize(N);
  nominalDualData_.projectedModelDataTrajectory.resize(N);

  if (N > 0) {
    // perform the computeRiccatiModificationTerms for partition i
    nextTimeIndex_ = 0;
    nextTaskId_ = 0;
    auto task = [this, N]() {
      int timeIndex;
      const matrix_t SmDummy = matrix_t::Zero(0, 0);

      // get next time index is atomic
      while ((timeIndex = nextTimeIndex_++) < N) {
        ComputeProjectionAndRiccatiModification(nominalPrimalData_.modelDataTrajectory[timeIndex], SmDummy,
                                                nominalDualData_.projectedModelDataTrajectory[timeIndex],
                                                nominalDualData_.riccatiModificationTrajectory[timeIndex]);
      }
    };
    RunParallel(task, Settings().nThreads_);
  }

  return SolveSequentialRiccatiEquationsImpl(finalValueFunction);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
matrix_t SLQ::ComputeHamiltonianHessian(const ModelData& modelData, const matrix_t& Sm) const {
  return searchStrategyPtr_->augmentHamiltonianHessian(modelData, modelData.cost.dfduu);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void SLQ::RiccatiEquationsWorker(size_t workerIndex, const std::pair<int, int>& partitionInterval,
                                 const ScalarFunctionQuadraticApproximation& finalValueFunction) {
  // set data for Riccati equations
  riccatiEquationsPtrStock_[workerIndex]->resetNumFunctionCalls();
  riccatiEquationsPtrStock_[workerIndex]->setData(
      &(nominalPrimalData_.primalSolution.timeTrajectory_), &(nominalDualData_.projectedModelDataTrajectory),
      &(nominalPrimalData_.primalSolution.postEventIndices_), &(nominalPrimalData_.modelDataEventTimes),
      &(nominalDualData_.riccatiModificationTrajectory));

  const auto& nominalTimeTrajectory = nominalPrimalData_.primalSolution.timeTrajectory_;
  const auto& nominalEventsPastTheEndIndices = nominalPrimalData_.primalSolution.postEventIndices_;

  auto& valueFunctionTrajectory = nominalDualData_.valueFunctionTrajectory;

  // Convert final value of value function in vector format
  vector_t allSsFinal = ContinuousTimeRiccatiEquations::Convert2Vector(finalValueFunction);

  scalar_array_t& SsNormalizedTime = SsNormalizedTimeTrajectoryStock_[workerIndex];
  SsNormalizedTime.clear();
  size_array_t& SsNormalizedPostEventIndices = SsNormalizedEventsPastTheEndIndecesStock_[workerIndex];
  SsNormalizedPostEventIndices.clear();

  /*
   *  The riccati equations are solved backwards in time
   *  the SsNormalized time is therefore filled with negative time in the reverse order, for example:
   *  nominalTime = [0.0, 1.0, 2.0, ..., 10.0]
   *  SsNormalized = [-10.0, ..., -2.0, -1.0, -0.0]
   */
  vector_array_t& allSsTrajectory = allSsTrajectoryStock_[workerIndex];
  IntegrateRiccatiEquationNominalTime(*riccatiIntegratorPtrStock_[workerIndex], *riccatiEquationsPtrStock_[workerIndex], partitionInterval,
                                      nominalTimeTrajectory, nominalEventsPastTheEndIndices, std::move(allSsFinal), SsNormalizedTime,
                                      SsNormalizedPostEventIndices, allSsTrajectory);

  // Convert value function to matrix format
  size_t outputN = SsNormalizedTime.size();
  for (size_t k = partitionInterval.first; k < partitionInterval.second; k++) {
    ContinuousTimeRiccatiEquations::Convert2Matrix(allSsTrajectory[outputN - 1 - k + partitionInterval.first], valueFunctionTrajectory[k]);
  }  // end of k loop
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void SLQ::IntegrateRiccatiEquationNominalTime(IntegratorBase& riccatiIntegrator, ContinuousTimeRiccatiEquations& riccatiEquation,
                                              const std::pair<int, int>& partitionInterval, const scalar_array_t& nominalTimeTrajectory,
                                              const size_array_t& nominalEventsPastTheEndIndices, vector_t allSsFinal,
                                              scalar_array_t& SsNormalizedTime, size_array_t& SsNormalizedPostEventIndices,
                                              vector_array_t& allSsTrajectory) {
  // normalized time and post event indices
  RetrieveActiveNormalizedTime(partitionInterval, nominalTimeTrajectory, nominalEventsPastTheEndIndices, SsNormalizedTime,
                               SsNormalizedPostEventIndices);
  // Extract sizes
  const int nominalTimeSize = SsNormalizedTime.size();
  const int numEvents = SsNormalizedPostEventIndices.size();
  auto partitionDuration = nominalTimeTrajectory[partitionInterval.second] - nominalTimeTrajectory[partitionInterval.first];

  // Normalized switching time indices, start and end of the partition are added at the beginning and end
  using iterator_t = scalar_array_t::const_iterator;
  std::vector<std::pair<iterator_t, iterator_t>> SsNormalizedSwitchingTimesIndices;
  SsNormalizedSwitchingTimesIndices.reserve(numEvents + 1);
  SsNormalizedSwitchingTimesIndices.emplace_back(SsNormalizedTime.cbegin(), SsNormalizedTime.cbegin());
  for (const auto& index : SsNormalizedPostEventIndices) {
    SsNormalizedSwitchingTimesIndices.back().second = SsNormalizedTime.cbegin() + index;
    SsNormalizedSwitchingTimesIndices.emplace_back(SsNormalizedTime.cbegin() + index, SsNormalizedTime.cbegin() + index);
  }
  SsNormalizedSwitchingTimesIndices.back().second = SsNormalizedTime.cend();

  // integrating the Riccati equations
  allSsTrajectory.clear();
  allSsTrajectory.reserve(nominalTimeSize);
  for (int i = 0; i <= numEvents; i++) {
    iterator_t beginTimeItr = SsNormalizedSwitchingTimesIndices[i].first;
    iterator_t endTimeItr = SsNormalizedSwitchingTimesIndices[i].second;

    // solve Riccati equations
    Observer Observer(&allSsTrajectory);
    const auto maxNumTimeSteps = static_cast<size_t>(Settings().maxNumStepsPerSecond_ * std::max(1.0, partitionDuration));
    riccatiIntegrator.integrateTimes(riccatiEquation, observer, allSsFinal, beginTimeItr, endTimeItr, Settings().timeStep_,
                                     Settings().absTolODE_, Settings().relTolODE_, maxNumTimeSteps);

    if (i < numEvents) {
      allSsFinal = riccatiEquation.computeJumpMap(*endTimeItr, allSsTrajectory.back());
    }
  }  // end of i loop

  // check size
  if (allSsTrajectory.size() != nominalTimeSize) {
    throw std::runtime_error("[SLQ::integrateRiccatiEquationNominalTim] allSsTrajectory size is incorrect.");
  }
}

}  // namespace bolza
