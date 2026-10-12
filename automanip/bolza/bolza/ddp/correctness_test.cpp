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

#include <gtest/gtest.h>
#include <cstdlib>
#include <ctime>
#include <iostream>

#include <bolza/qp/qp_solver.hpp>
#include <bolza/qp/qp_discrete_transcription.hpp>
#include <bolza/qp/qp_solver.hpp>
#include "helpers/problems_generation_test.hpp"

#include <bolza/core/initialization/operating_points.hpp>
#include <bolza/oc/rollout/performance_indices_rollout.hpp>
#include <bolza/oc/rollout/time_triggered_rollout.hpp>
#include "helpers/problems_generation_test.hpp"

#include <bolza/ddp/ilqr.hpp>
#include <bolza/ddp/slq.hpp>

enum class NumThreads { SINGLE, MULTI };
enum class Constraining { CONSTARINED, UNCONSTRAINED };

class DDPCorrectness : public testing::TestWithParam<std::tuple<bolza::search_strategy::Type, Constraining, NumThreads>> {
 protected:
  static constexpr size_t N = 50;
  static constexpr size_t STATE_DIM = 3;
  static constexpr size_t INPUT_DIM = 2;
  static constexpr bolza::scalar_t minRelCost = 1e-3;
  static constexpr bolza::scalar_t solutionPrecision = 5e-3;
  static constexpr size_t numStateInputConstraints = 2;

  DDPCorrectness() {
    srand(0);

    // Try generateing problem
    for (int retries = 0; !createFeasibleRandomProblem(); retries++) {
      std::cerr << "Random problem was infeasible, retry ...\n";
      if (retries > 10) {
        throw std::runtime_error("Failed generating feasible problem");
      }
    }

    qpSolution = bolza::qp_solver::SolveLinearQuadraticOptimalControlProblem(*problemPtr, nominalTrajectory, initState);
    qpCost = getQpCost(qpSolution);

    // system operating points
    bolza::vector_array_t inputTrajectoryTemp(N);
    std::copy(nominalTrajectory.inputTrajectory.begin(), nominalTrajectory.inputTrajectory.end(), inputTrajectoryTemp.begin());
    inputTrajectoryTemp.push_back(inputTrajectoryTemp.back());
    operatingPointsPtr.Reset(
        new bolza::OperatingPoints(nominalTrajectory.timeTrajectory, nominalTrajectory.stateTrajectory, inputTrajectoryTemp));

    // rollout settings
    const bolza::Rollout::Settings rolloutSettings = []() {
      bolza::Rollout::Settings rolloutSettings;
      rolloutSettings.absTolODE = 1e-10;
      rolloutSettings.relTolODE = 1e-7;
      rolloutSettings.timeStep = 1e-3;
      rolloutSettings.maxNumStepsPerSecond = 10000;
      return rolloutSettings;
    }();

    // rollout
    rolloutPtr.Reset(new bolza::TimeTriggeredRollout(*systemPtr, rolloutSettings));

    startTime = nominalTrajectory.timeTrajectory.front();
    finalTime = nominalTrajectory.timeTrajectory.back();
  }

  bool createFeasibleRandomProblem() {
    // dynamics
    systemPtr = bolza::GetOcs2Dynamics(bolza::GetRandomDynamics(STATE_DIM, INPUT_DIM));
    problemPtr.Reset(new bolza::OptimalControlProblem);
    problemPtr->dynamicsPtr.Reset(systemPtr->Clone());

    // cost
    problemPtr->costPtr->Add("cost", bolza::GetOcs2Cost(bolza::GetRandomCost(STATE_DIM, INPUT_DIM)));
    problemPtr->finalCostPtr->Add("finalCost", bolza::GetOcs2StateCost(bolza::GetRandomCost(STATE_DIM, 0)));
    targetTrajectories = bolza::TargetTrajectories({0.0}, {bolza::vector_t::Random(STATE_DIM)}, {bolza::vector_t::Random(INPUT_DIM)});

    // constraint
    if (std::get<1>(GetParam()) == Constraining::CONSTARINED) {
      problemPtr->equalityConstraintPtr->Add(
          "equality", bolza::GetOcs2Constraints(bolza::GetRandomConstraints(STATE_DIM, INPUT_DIM, numStateInputConstraints)));
    }

    // system operating points
    nominalTrajectory = bolza::qp_solver::GetRandomTrajectory(N, STATE_DIM, INPUT_DIM, 1e-3);
    initState = bolza::vector_t::Random(STATE_DIM);

    // get QP
    problemPtr->targetTrajectoriesPtr = &targetTrajectories;
    bolza::ScalarFunctionQuadraticApproximation qpCosts;
    bolza::VectorFunctionLinearApproximation qpConstraints;
    const auto lqApproximation = GetLinearQuadraticApproximation(*problemPtr, nominalTrajectory);
    const bolza::vector_t dx0 = initState - nominalTrajectory.stateTrajectory.front();
    std::tie(qpCosts, qpConstraints) = GetDenseQp(lqApproximation, dx0);

    // Check feasibility
    if (!bolza::qp_solver::IsQpFeasible(qpCosts, qpConstraints)) {
      return false;
    }

    // Correct the nominal trajectory to not violate the constraints.
    nominalTrajectory = getFeasibleTrajectory(qpConstraints, nominalTrajectory);

    return true;
  }

  /** Modifies given trajectory to satisfy the constraints */
  bolza::qp_solver::ContinuousTrajectory getFeasibleTrajectory(const bolza::VectorFunctionLinearApproximation& qpConstraints,
                                                              const bolza::qp_solver::ContinuousTrajectory& trajectory) const {
    const auto& A = qpConstraints.dfdx;  // A w + b = 0,  A must be full row-rank such that (A A') is invertible
    const auto& b = qpConstraints.f;     // b = [x0; e[0]; b[0]; ... e[N-1]; b[N-1]; e[N]]

    /* Find the trajectory correction w to satisfy the constraint by solving
     *   min  1/2 w' w
     *   s.t. A w + b = 0  */
    const bolza::vector_t w = -A.transpose() * (A * A.transpose()).inverse() * b;  // w = [dx[0], du[0], dx[1],  du[1], ..., dx[N]]

    // Make trajectory feasible
    auto feasibleTrajectory = trajectory;
    int nextIndex = 0;
    const auto N = feasibleTrajectory.inputTrajectory.size();
    for (int k = 0; k < N; k++) {
      const auto nx = feasibleTrajectory.stateTrajectory[k].size();
      const auto nu = feasibleTrajectory.inputTrajectory[k].size();
      feasibleTrajectory.stateTrajectory[k] += w.segment(nextIndex, nx);       // dx[k]
      feasibleTrajectory.inputTrajectory[k] += w.segment(nextIndex + nx, nu);  // du[k]
      nextIndex += nx + nu;
    }
    feasibleTrajectory.stateTrajectory[N] += w.segment(nextIndex, feasibleTrajectory.stateTrajectory[N].size());  // dx[N]

    return feasibleTrajectory;
  }

  bolza::search_strategy::Type getSearchStrategy() { return std::get<0>(GetParam()); }

  size_t getNumThreads() const {
    const auto n = std::get<2>(GetParam());
    if (n == NumThreads::SINGLE) {
      return 1;
    } else {
      return 2;
    }
  }

  bolza::ddp::Settings getSettings(bolza::ddp::Algorithm algorithmType, size_t numThreads, bolza::search_strategy::Type strategy,
                                  bool display = false) const {
    bolza::ddp::Settings ddpSettings;
    ddpSettings.algorithm_ = algorithmType;
    ddpSettings.displayInfo_ = false;
    ddpSettings.displayShortSummary_ = display;
    ddpSettings.absTolODE_ = 1e-10;
    ddpSettings.relTolODE_ = 1e-7;
    ddpSettings.maxNumStepsPerSecond_ = 10000;
    ddpSettings.minRelCost_ = minRelCost;
    ddpSettings.nThreads_ = numThreads;
    ddpSettings.maxNumIterations_ = 2 + (numThreads - 1);  // need an extra iteration for each added time partition
    ddpSettings.strategy_ = strategy;
    ddpSettings.lineSearch_.minStepLength = 1e-4;
    return ddpSettings;
  }

  bolza::scalar_t getQpCost(const bolza::qp_solver::ContinuousTrajectory& qpSolution) const {
    auto costFunc = [this](bolza::scalar_t t, const bolza::vector_t& x, const bolza::vector_t& u) {
      return problemPtr->costPtr->GetValue(t, x, u, targetTrajectories, bolza::PreComputation());
    };
    auto inputTrajectoryTemp = qpSolution.inputTrajectory;
    inputTrajectoryTemp.emplace_back(inputTrajectoryTemp.back());
    auto lAccum =
        bolza::PerformanceIndicesRollout::RolloutCost(costFunc, qpSolution.timeTrajectory, qpSolution.stateTrajectory, inputTrajectoryTemp);

    return lAccum + problemPtr->finalCostPtr->GetValue(qpSolution.timeTrajectory.back(), qpSolution.stateTrajectory.back(),
                                                       targetTrajectories, bolza::PreComputation());
  }

  std::string getTestName(const bolza::ddp::Settings& ddpSettings) const {
    std::string testName;
    testName += "Correctness Test { ";
    testName += "Algorithm: " + bolza::ddp::ToAlgorithmName(ddpSettings.algorithm_) + ",  ";
    testName += "Strategy: " + bolza::search_strategy::ToString(ddpSettings.strategy_) + ",  ";
    testName += "#threads: " + std::to_string(ddpSettings.nThreads_) + " }";
    return testName;
  }

  void correctnessTest(const bolza::ddp::Settings& ddpSettings, const bolza::PerformanceIndex& performanceIndex,
                       const bolza::PrimalSolution& ddpSolution) const {
    const auto testName = getTestName(ddpSettings);
    EXPECT_NEAR(performanceIndex.cost, qpCost, 10.0 * minRelCost) << "MESSAGE: " << testName << ": failed in the optimal cost test!";
    EXPECT_LT(relError(ddpSolution.stateTrajectory_.back(), qpSolution.stateTrajectory.back()), solutionPrecision)
        << "MESSAGE: " << testName << ": failed in the optimal final state test!";
    EXPECT_LT(relError(ddpSolution.inputTrajectory_.front(), qpSolution.inputTrajectory.front()), solutionPrecision)
        << "MESSAGE: " << testName << ": failed in the optimal initial input test!";
  }

  bolza::scalar_t relError(bolza::vector_t ddpSol, const bolza::vector_t& qpSol) const { return (ddpSol - qpSol).norm() / ddpSol.norm(); }

  bolza::vector_t initState;
  bolza::scalar_t startTime;
  bolza::scalar_t finalTime;

  bolza::TargetTrajectories targetTrajectories;
  std::unique_ptr<bolza::SystemDynamicsBase> systemPtr;
  std::unique_ptr<bolza::OptimalControlProblem> problemPtr;
  std::unique_ptr<bolza::OperatingPoints> operatingPointsPtr;
  bolza::qp_solver::ContinuousTrajectory nominalTrajectory;
  std::unique_ptr<bolza::TimeTriggeredRollout> rolloutPtr;

  bolza::scalar_t qpCost;
  bolza::qp_solver::ContinuousTrajectory qpSolution;
};

constexpr size_t DDPCorrectness::N;
constexpr size_t DDPCorrectness::STATE_DIM;
constexpr size_t DDPCorrectness::INPUT_DIM;
constexpr bolza::scalar_t DDPCorrectness::minRelCost;
constexpr bolza::scalar_t DDPCorrectness::solutionPrecision;
constexpr size_t DDPCorrectness::numStateInputConstraints;

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
TEST_P(DDPCorrectness, TestSLQ) {
  // settings
  const auto ddpSettings = getSettings(bolza::ddp::Algorithm::SLQ, getNumThreads(), getSearchStrategy());

  // ddp
  bolza::SLQ ddp(ddpSettings, *rolloutPtr, *problemPtr, *operatingPointsPtr);

  ddp.GetReferenceManager().SetTargetTrajectories(targetTrajectories);
  ddp.Run(startTime, initState, finalTime);
  const auto performanceIndex = ddp.GetPerformanceIndeces();
  const auto solution = ddp.PrimalSolution(finalTime);

  correctnessTest(ddpSettings, performanceIndex, solution);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
TEST_P(DDPCorrectness, TestILQR) {
  // settings
  const auto ddpSettings = getSettings(bolza::ddp::Algorithm::ILQR, getNumThreads(), getSearchStrategy());

  // ddp
  bolza::ILQR ddp(ddpSettings, *rolloutPtr, *problemPtr, *operatingPointsPtr);

  ddp.GetReferenceManager().SetTargetTrajectories(targetTrajectories);
  ddp.Run(startTime, initState, finalTime);
  const auto performanceIndex = ddp.GetPerformanceIndeces();
  const auto solution = ddp.PrimalSolution(finalTime);

  correctnessTest(ddpSettings, performanceIndex, solution);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
/* Test name printed in gtest results */
std::string testName(const testing::TestParamInfo<DDPCorrectness::ParamType>& info) {
  std::string name;
  name += bolza::search_strategy::ToString(std::get<0>(info.param)) + "__";
  name += std::get<1>(info.param) == Constraining::CONSTARINED ? "CONSTARINED" : "UNCONSTRAINED";
  name += "__";
  name += std::get<2>(info.param) == NumThreads::SINGLE ? "SINGLE_THREAD" : "MULTI_THREAD";
  return name;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
INSTANTIATE_TEST_CASE_P(DDPCorrectnessTestCase, DDPCorrectness,
                        testing::Combine(testing::ValuesIn({bolza::search_strategy::Type::LINE_SEARCH,
                                                            bolza::search_strategy::Type::LEVENBERG_MARQUARDT}),
                                         testing::ValuesIn({Constraining::CONSTARINED, Constraining::UNCONSTRAINED}),
                                         testing::ValuesIn({NumThreads::SINGLE, NumThreads::MULTI})),
                        testName);
