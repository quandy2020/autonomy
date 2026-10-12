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

#include "bolza/ddp/ilqr.hpp"
#include <bolza/ddp/riccati_equations/riccati_transversality_conditions.hpp>

namespace bolza {

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
ILQR::ILQR(ddp::Settings ddpSettings, const RolloutBase& rollout, const OptimalControlProblem& optimalControlProblem,
           const Initializer& initializer)
    : GaussNewtonDDP(std::move(ddpSettings), rollout, optimalControlProblem, initializer) {
  if (Settings().algorithm_ != ddp::Algorithm::ILQR) {
    throw std::runtime_error("[ILQR] In DDP setting the algorithm name is set \"" + ddp::ToAlgorithmName(Settings().algorithm_) +
                             "\" while ILQR is instantiated!");
  }

  // dynamics discretizer
  sensitivityDiscretizer_ = [&]() {
    switch (Settings().backwardPassIntegratorType_) {
      case IntegratorType::EULER:
        return SelectDynamicsSensitivityDiscretization(SensitivityIntegratorType::EULER);
      case IntegratorType::RK4:
        return SelectDynamicsSensitivityDiscretization(SensitivityIntegratorType::RK4);
      case IntegratorType::ODE45:
        return SelectDynamicsSensitivityDiscretization(SensitivityIntegratorType::RK4);
      case IntegratorType::ODE45_OCS2:
        return SelectDynamicsSensitivityDiscretization(SensitivityIntegratorType::RK4);
      default:
        throw std::runtime_error("[ILQR] Integrator of type " + integrator_type::ToString(Settings().backwardPassIntegratorType_) +
                                 " is not supported for sensitivity discretization! Modify ddp::Settings::backwardPassIntegratorType_.");
    }
  }();

  // Riccati solver
  riccatiEquationsPtrStock_.clear();
  riccatiEquationsPtrStock_.reserve(Settings().nThreads_);
  for (size_t i = 0; i < Settings().nThreads_; i++) {
    const bool isRiskSensitive = !numerics::almost_eq(Settings().riskSensitiveCoeff_, 0.0);
    const bool preComputeRiccatiTerms = Settings().preComputeRiccatiTerms_ && (Settings().strategy_ == search_strategy::Type::LINE_SEARCH);
    riccatiEquationsPtrStock_.emplace_back(new DiscreteTimeRiccatiEquations(preComputeRiccatiTerms, isRiskSensitive));
    riccatiEquationsPtrStock_.back()->setRiskSensitiveCoefficient(Settings().riskSensitiveCoeff_);
  }  // end of i loop

  Eigen::initParallel();
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void ILQR::ApproximateIntermediateLQ(const DualSolution& dualSolution, PrimalDataContainer& primalData) {
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
    size_t taskId = nextTaskId_++;  // assign task ID (atomic)

    ModelData continuousTimeModelData;

    // get next time index is atomic
    size_t timeIndex;
    while ((timeIndex = nextTimeIndex_++) < timeTrajectory.size()) {
      // approximate continuous LQ for the given time index
      bolza::ApproximateIntermediateLQ(optimalControlProblemStock_[taskId], timeTrajectory[timeIndex], stateTrajectory[timeIndex],
                                      inputTrajectory[timeIndex], multiplierTrajectory[timeIndex], continuousTimeModelData);

      // checking the numerical properties
      if (Settings().checkNumericalStability_) {
        const auto errSize = CheckSize(continuousTimeModelData, stateTrajectory[timeIndex].rows(), inputTrajectory[timeIndex].rows());
        if (!errSize.empty()) {
          throw std::runtime_error("[ILQR::ApproximateIntermediateLQ] Mismatch in dimensions at intermediate time: " +
                                   std::to_string(timeTrajectory[timeIndex]) + "\n" + errSize);
        }
        const auto errProperties = CheckDynamicsProperties(continuousTimeModelData) + CheckCostProperties(continuousTimeModelData) +
                                   CheckConstraintProperties(continuousTimeModelData);
        if (!errProperties.empty()) {
          throw std::runtime_error("[ILQR::ApproximateIntermediateLQ] Ill-posed problem at intermediate time: " +
                                   std::to_string(timeTrajectory[timeIndex]) + "\n" + errProperties);
        }
      }

      // discretize LQ problem
      const scalar_t timeStep = (timeIndex + 1 < timeTrajectory.size()) ? (timeTrajectory[timeIndex + 1] - timeTrajectory[timeIndex]) : 0.0;
      if (!numerics::almost_eq(timeStep, 0.0)) {
        DiscreteLQWorker(*optimalControlProblemStock_[taskId].dynamicsPtr, timeTrajectory[timeIndex], stateTrajectory[timeIndex],
                         inputTrajectory[timeIndex], timeStep, continuousTimeModelData, modelDataTrajectory[timeIndex]);
      } else {
        modelDataTrajectory[timeIndex] = continuousTimeModelData;
      }
    }
  };

  RunParallel(task, Settings().nThreads_);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void ILQR::DiscreteLQWorker(SystemDynamicsBase& system, scalar_t time, const vector_t& state, const vector_t& input, scalar_t timeStep,
                            const ModelData& continuousTimeModelData, ModelData& modelData) {
  modelData.time = continuousTimeModelData.time;
  modelData.stateDim = continuousTimeModelData.stateDim;
  modelData.inputDim = continuousTimeModelData.inputDim;

  // linearize system dynamics
  modelData.dynamicsBias.setZero(modelData.stateDim);
  modelData.dynamics = sensitivityDiscretizer_(system, time, state, input, timeStep);
  modelData.dynamics.f.setZero(modelData.stateDim);

  // quadratic approximation to the cost function
  modelData.cost = continuousTimeModelData.cost;
  modelData.cost *= timeStep;

  // state equality constraints
  modelData.stateEqConstraint = continuousTimeModelData.stateEqConstraint;

  // state-input equality constraints
  modelData.stateInputEqConstraint = continuousTimeModelData.stateInputEqConstraint;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void ILQR::CalculateControllerWorker(size_t timeIndex, const PrimalDataContainer& primalData, const DualDataContainer& dualData,
                                     LinearController& dstController) {
  const auto& nominalState = primalData.primalSolution.stateTrajectory_[timeIndex];
  const auto& nominalInput = primalData.primalSolution.inputTrajectory_[timeIndex];

  const auto& EvProjected = dualData.projectedModelDataTrajectory[timeIndex].stateInputEqConstraint.f;
  const auto& CmProjected = dualData.projectedModelDataTrajectory[timeIndex].stateInputEqConstraint.dfdx;

  const auto& Qu = dualData.riccatiModificationTrajectory[timeIndex].constraintNullProjector_;

  // feedback gains
  dstController.gainArray_[timeIndex] = -CmProjected;
  dstController.gainArray_[timeIndex].noalias() += Qu * projectedKmTrajectoryStock_[timeIndex];

  // bias input
  dstController.biasArray_[timeIndex] = nominalInput;
  dstController.biasArray_[timeIndex].noalias() -= dstController.gainArray_[timeIndex] * nominalState;
  dstController.deltaBiasArray_[timeIndex] = -EvProjected;
  dstController.deltaBiasArray_[timeIndex].noalias() += Qu * projectedLvTrajectoryStock_[timeIndex];
}

/******************************************************************************************************/
/******************************************************************************************************/
/***************************************************************************************************** */
scalar_t ILQR::SolveSequentialRiccatiEquations(const ScalarFunctionQuadraticApproximation& finalValueFunction) {
  const size_t N = nominalPrimalData_.primalSolution.timeTrajectory_.size();
  projectedLvTrajectoryStock_.resize(N);
  projectedKmTrajectoryStock_.resize(N);

  nominalDualData_.riccatiModificationTrajectory.resize(N);
  nominalDualData_.projectedModelDataTrajectory.resize(N);

  const auto& finalModelData = nominalPrimalData_.modelDataTrajectory.back();
  auto& finalRiccatiModification = nominalDualData_.riccatiModificationTrajectory.back();
  auto& finalProjectedModelData = nominalDualData_.projectedModelDataTrajectory.back();
  auto& finalProjectedLvFinal = projectedLvTrajectoryStock_.back();
  auto& finalProjectedKmFinal = projectedKmTrajectoryStock_.back();

  const matrix_t SmDummy = matrix_t::Zero(finalModelData.stateDim, finalModelData.stateDim);
  ComputeProjectionAndRiccatiModification(finalModelData, SmDummy, finalProjectedModelData, finalRiccatiModification);

  // projected feedforward
  finalProjectedLvFinal = -finalProjectedModelData.cost.dfdu - finalRiccatiModification.deltaGv_;
  finalProjectedLvFinal.noalias() -= finalProjectedModelData.dynamics.dfdu.transpose() * finalValueFunction.dfdx;

  // projected feedback
  finalProjectedKmFinal = -finalProjectedModelData.cost.dfdux - finalRiccatiModification.deltaGm_;
  finalProjectedKmFinal.noalias() -= finalProjectedModelData.dynamics.dfdu.transpose() * finalValueFunction.dfdxx;

  return SolveSequentialRiccatiEquationsImpl(finalValueFunction);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
matrix_t ILQR::ComputeHamiltonianHessian(const ModelData& modelData, const matrix_t& Sm) const {
  const matrix_t BmTransSm = modelData.dynamics.dfdu.transpose() * Sm;
  matrix_t Hm = modelData.cost.dfduu;
  Hm.noalias() += BmTransSm * modelData.dynamics.dfdu;
  return searchStrategyPtr_->augmentHamiltonianHessian(modelData, Hm);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void ILQR::RiccatiEquationsWorker(size_t workerIndex, const std::pair<int, int>& partitionInterval,
                                  const ScalarFunctionQuadraticApproximation& finalValueFunction) {
  // find all events belonging to the current partition
  const auto& postEventIndices = nominalPrimalData_.primalSolution.postEventIndices_;
  const auto firstEventItr = std::upper_bound(postEventIndices.begin(), postEventIndices.end(), partitionInterval.first);
  const auto lastEventItr = std::upper_bound(postEventIndices.begin(), postEventIndices.end(), partitionInterval.second);

  // final temporal values. Used to store pre-jump value
  ScalarFunctionQuadraticApproximation finalValueTemp = finalValueFunction;

  /*
   * solving the Riccati equations
   */
  const ScalarFunctionQuadraticApproximation* valueFunctionNext = &finalValueTemp;

  int curIndex = partitionInterval.second - 1;
  auto nextEventItr = lastEventItr - 1;
  const int stopIndex = partitionInterval.first;
  while (curIndex >= stopIndex) {
    auto& curProjectedLv = projectedLvTrajectoryStock_[curIndex];
    auto& curProjectedKm = projectedKmTrajectoryStock_[curIndex];
    auto& curProjectedModelData = nominalDualData_.projectedModelDataTrajectory[curIndex];
    auto& curRiccatiModification = nominalDualData_.riccatiModificationTrajectory[curIndex];
    const auto& curModelData = nominalPrimalData_.modelDataTrajectory[curIndex];

    auto& curSm = nominalDualData_.valueFunctionTrajectory[curIndex].dfdxx;
    auto& curSv = nominalDualData_.valueFunctionTrajectory[curIndex].dfdx;
    auto& curs = nominalDualData_.valueFunctionTrajectory[curIndex].f;

    ComputeProjectionAndRiccatiModification(curModelData, valueFunctionNext->dfdxx, curProjectedModelData, curRiccatiModification);

    riccatiEquationsPtrStock_[workerIndex]->computeMap(curProjectedModelData, curRiccatiModification, valueFunctionNext->dfdxx,
                                                       valueFunctionNext->dfdx, valueFunctionNext->f, curProjectedKm, curProjectedLv, curSm,
                                                       curSv, curs);
    valueFunctionNext = &(nominalDualData_.valueFunctionTrajectory[curIndex]);

    if (std::Distance(firstEventItr, nextEventItr) >= 0 && curIndex == *nextEventItr) {
      // move to pre-event index
      --curIndex;

      const int index = std::Distance(postEventIndices.begin(), nextEventItr);
      std::tie(finalValueTemp.dfdxx, finalValueTemp.dfdx, finalValueTemp.f) =
          RiccatiTransversalityConditions(nominalPrimalData_.modelDataEventTimes[index], curSm, curSv, curs);

      nominalDualData_.valueFunctionTrajectory[curIndex] = finalValueTemp;

      const auto& finalModelData = nominalPrimalData_.modelDataTrajectory[curIndex];
      auto& finalRiccatiModification = nominalDualData_.riccatiModificationTrajectory[curIndex];
      auto& finalProjectedModelData = nominalDualData_.projectedModelDataTrajectory[curIndex];
      auto& finalProjectedLvFinal = projectedLvTrajectoryStock_[curIndex];
      auto& finalProjectedKmFinal = projectedKmTrajectoryStock_[curIndex];

      const matrix_t SmDummy = matrix_t::Zero(finalModelData.stateDim, finalModelData.stateDim);
      ComputeProjectionAndRiccatiModification(finalModelData, SmDummy, finalProjectedModelData, finalRiccatiModification);

      // projected feedforward
      finalProjectedLvFinal = -finalProjectedModelData.cost.dfdu - finalRiccatiModification.deltaGv_;
      finalProjectedLvFinal.noalias() -=
          finalProjectedModelData.dynamics.dfdu.transpose() * nominalDualData_.valueFunctionTrajectory[curIndex].dfdx;

      // projected feedback
      finalProjectedKmFinal = -finalProjectedModelData.cost.dfdux - finalRiccatiModification.deltaGm_;
      finalProjectedKmFinal.noalias() -=
          finalProjectedModelData.dynamics.dfdu.transpose() * nominalDualData_.valueFunctionTrajectory[curIndex].dfdxx;

      valueFunctionNext = &finalValueTemp;

      --nextEventItr;
    }

    --curIndex;
  }  // while
}
}  // namespace bolza
