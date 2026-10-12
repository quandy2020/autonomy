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

#include <bolza/core/control/feedforward_controller.hpp>
#include <bolza/core/initialization/default_initializer.hpp>
#include <bolza/oc/rollout/time_triggered_rollout.hpp>
#include "helpers/exp1.hpp"

#include <bolza/ddp/ilqr.hpp>
#include <bolza/ddp/slq.hpp>

class Exp1 : public testing::TestWithParam<std::tuple<bolza::search_strategy::Type, size_t>> {
 protected:
  static constexpr size_t STATE_DIM = 2;
  static constexpr size_t INPUT_DIM = 1;
  static constexpr bolza::scalar_t timeStep = 1e-2;
  static constexpr bolza::scalar_t minRelCost = 1e-3;
  static constexpr bolza::scalar_t expectedCost = 5.4399;

  Exp1() {
    // event times
    const bolza::scalar_array_t eventTimes{0.2262, 1.0176};
    const std::vector<size_t> modeSequence{0, 1, 2};
    referenceManagerPtr = bolza::GetExp1ReferenceManager(eventTimes, modeSequence);

    problem = bolza::CreateExp1Problem(referenceManagerPtr);

    // operatingTrajectories
    initializerPtr.Reset(new bolza::DefaultInitializer(INPUT_DIM));
  }

  bolza::search_strategy::Type getSearchStrategy() { return std::get<0>(GetParam()); }

  size_t getNumThreads() { return std::get<1>(GetParam()); }

  // rollout settings
  bolza::Rollout::Settings RolloutSettings() const {
    bolza::Rollout::Settings rolloutSettings;
    rolloutSettings.absTolODE = 1e-10;
    rolloutSettings.relTolODE = 1e-7;
    rolloutSettings.timeStep = timeStep;
    rolloutSettings.integratorType = bolza::IntegratorType::ODE45;
    rolloutSettings.maxNumStepsPerSecond = 10000;
    return rolloutSettings;
  };

  bolza::ddp::Settings getSettings(bolza::ddp::Algorithm algorithmType, size_t numThreads, bolza::search_strategy::Type strategy,
                                  bool display = false) const {
    bolza::ddp::Settings ddpSettings;
    ddpSettings.algorithm_ = algorithmType;
    ddpSettings.nThreads_ = numThreads;
    ddpSettings.preComputeRiccatiTerms_ = false;
    ddpSettings.displayInfo_ = false;
    ddpSettings.displayShortSummary_ = display;
    ddpSettings.maxNumIterations_ = 30;
    ddpSettings.minRelCost_ = minRelCost;
    ddpSettings.checkNumericalStability_ = true;
    ddpSettings.absTolODE_ = 1e-10;
    ddpSettings.relTolODE_ = 1e-7;
    ddpSettings.timeStep_ = timeStep;
    ddpSettings.backwardPassIntegratorType_ = bolza::IntegratorType::ODE45;
    ddpSettings.maxNumStepsPerSecond_ = 10000;
    ddpSettings.useFeedbackPolicy_ = false;
    ddpSettings.debugPrintRollout_ = false;
    ddpSettings.strategy_ = strategy;
    ddpSettings.lineSearch_.minStepLength = 0.0001;
    return ddpSettings;
  }

  std::string getTestName(const bolza::ddp::Settings& ddpSettings) const {
    std::string testName;
    testName += "EXP1 Test { ";
    testName += "Algorithm: " + bolza::ddp::ToAlgorithmName(ddpSettings.algorithm_) + ",  ";
    testName += "Strategy: " + bolza::search_strategy::ToString(ddpSettings.strategy_) + ",  ";
    testName += "#threads: " + std::to_string(ddpSettings.nThreads_) + " }";
    return testName;
  }

  void performanceIndexTest(const bolza::ddp::Settings& ddpSettings, const bolza::PerformanceIndex& performanceIndex) const {
    const auto testName = getTestName(ddpSettings);
    EXPECT_NEAR(performanceIndex.cost, expectedCost, 10.0 * minRelCost) << "MESSAGE: " << testName << ": failed in the total cost test!";
    EXPECT_NEAR(performanceIndex.equalityConstraintsSSE, 0.0, 10.0 * ddpSettings.constraintTolerance_)
        << "MESSAGE: " << testName << ": failed in state-input equality constraint ISE test!";
  }

  const bolza::scalar_t startTime = 0.0;
  const bolza::scalar_t finalTime = 3.0;
  const bolza::vector_t initState = (bolza::vector_t(STATE_DIM) << 2.0, 3.0).finished();
  std::shared_ptr<bolza::ReferenceManager> referenceManagerPtr;

  bolza::OptimalControlProblem problem;
  std::unique_ptr<bolza::Initializer> initializerPtr;
};

constexpr size_t Exp1::STATE_DIM;
constexpr size_t Exp1::INPUT_DIM;
constexpr bolza::scalar_t Exp1::timeStep;
constexpr bolza::scalar_t Exp1::minRelCost;
constexpr bolza::scalar_t Exp1::expectedCost;

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
TEST_F(Exp1, ddp_hamiltonian) {
  // ddp settings
  auto ddpSettings = getSettings(bolza::ddp::Algorithm::SLQ, 2, bolza::search_strategy::Type::LINE_SEARCH);
  ddpSettings.useFeedbackPolicy_ = true;
  ddpSettings.maxNumIterations_ = 50;
  ddpSettings.minRelCost_ = 1e-9;  // to allow more iterations that the effect of final linesearch is negligible

  // dynamics and rollout
  bolza::EXP1_System systemDynamics(referenceManagerPtr);
  bolza::TimeTriggeredRollout Rollout(systemDynamics, RolloutSettings());

  // instantiate
  bolza::SLQ ddp(ddpSettings, rollout, problem, *initializerPtr);
  ddp.SetReferenceManager(referenceManagerPtr);

  // run ddp
  ddp.Run(startTime, initState, finalTime);
  // get solution
  const auto solution = ddp.PrimalSolution(finalTime);

  // define precision for tests
  constexpr bolza::scalar_t precision = 1e-3;
  // note: in the following highly non-linear system means more than quadratic in the state

  // get Hamiltonian at current solution
  // expected outcome: true, because the current solution should be optimal
  bolza::scalar_t time = solution.timeTrajectory_.front();
  bolza::vector_t state = solution.stateTrajectory_.front();
  bolza::vector_t input = solution.controllerPtr_->ComputeInput(time, state);
  auto hamiltonian = ddp.GetHamiltonian(time, state, input);
  const bolza::vector_t dHdu1a = hamiltonian.dfdu;
  EXPECT_TRUE(dHdu1a.isZero(precision)) << "MESSAGE for test 1a: Derivative of Hamiltonian w.r.t. to u is not zero: " << dHdu1a.transpose();

  // evaluate Hamiltonian at different state (but using feedback policy)
  // expected outcome: true, because for a highly non-linear system the LQ approximation of H is not exact,
  //                         however since the subsystems' dynamics are control-affine and the cost is quadratic w.r.t. to input,
  //                         this looks like a LQR problem and the linear feedback policy appears to be globally optimal
  bolza::scalar_t querryTime = solution.timeTrajectory_.front();
  bolza::vector_t querryState = bolza::vector_t::Random(solution.stateTrajectory_.front().size());
  bolza::vector_t querryInput = solution.controllerPtr_->ComputeInput(querryTime, querryState);
  const bolza::vector_t dHdu1b = hamiltonian.dfdux * (querryState - state) + hamiltonian.dfduu * (querryInput - input) + hamiltonian.dfdu;
  EXPECT_TRUE(dHdu1b.isZero(precision)) << "MESSAGE for test 1b: Derivative of Hamiltonian w.r.t. to u is not zero: " << dHdu1b.transpose();

  // evaluate Hamiltonian at different input
  // expected outcome: false, because for a highly non-linear system the LQ approximation of H is not exact
  //                          but still a random input does not appear to be optimal
  querryTime = solution.timeTrajectory_.front();
  querryState = solution.stateTrajectory_.front();
  querryInput = bolza::vector_t::Random(solution.inputTrajectory_.front().size());
  const bolza::vector_t dHdu1c = hamiltonian.dfdux * (querryState - state) + hamiltonian.dfduu * (querryInput - input) + hamiltonian.dfdu;
  EXPECT_FALSE(dHdu1c.isZero(precision)) << "MESSAGE for test 1c: Derivative of Hamiltonian w.r.t. to u is zero: " << dHdu1c.transpose();

  // get Hamiltonian at different state (but using feedback policy)
  // expected outcome: false, because for a highly non-linear system the linear feedback policy is not globally optimal
  time = solution.timeTrajectory_.front();
  state = bolza::vector_t::Random(solution.stateTrajectory_.front().size());
  input = solution.controllerPtr_->ComputeInput(time, state);
  hamiltonian = ddp.GetHamiltonian(time, state, input);
  const bolza::vector_t dHdu2 = hamiltonian.dfdu;
  EXPECT_FALSE(dHdu2.isZero(precision)) << "MESSAGE for test 2: Derivative of Hamiltonian w.r.t. to u is zero: " << dHdu2.transpose();

  // get Hamiltonian at different input
  // expected outcome: false, because a random input is not optimal
  time = solution.timeTrajectory_.front();
  state = solution.stateTrajectory_.front();
  input = bolza::vector_t::Random(solution.inputTrajectory_.front().size());
  hamiltonian = ddp.GetHamiltonian(time, state, input);
  const bolza::vector_t dHdu3 = hamiltonian.dfdu;
  EXPECT_FALSE(dHdu3.isZero(precision)) << "MESSAGE for test 3: Derivative of Hamiltonian w.r.t. to u is zero: " << dHdu3.transpose();
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
TEST_P(Exp1, SLQ) {
  // ddp settings
  const auto ddpSettings = getSettings(bolza::ddp::Algorithm::SLQ, getNumThreads(), getSearchStrategy());

  // dynamics and rollout
  bolza::EXP1_System systemDynamics(referenceManagerPtr);
  bolza::TimeTriggeredRollout Rollout(systemDynamics, RolloutSettings());

  // instantiate
  bolza::SLQ ddp(ddpSettings, rollout, problem, *initializerPtr);
  ddp.SetReferenceManager(referenceManagerPtr);

  if (ddpSettings.displayInfo_ || ddpSettings.displayShortSummary_) {
    std::cerr << "\n" << getTestName(ddpSettings) << "\n";
  }

  // run ddp
  ddp.Run(startTime, initState, finalTime);
  // get performance index
  const auto performanceIndex = ddp.GetPerformanceIndeces();

  // performanceIndeces test
  performanceIndexTest(ddpSettings, performanceIndex);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
TEST_P(Exp1, ILQR) {
  // ddp settings
  const auto ddpSettings = getSettings(bolza::ddp::Algorithm::ILQR, getNumThreads(), getSearchStrategy());

  // dynamics and rollout
  bolza::EXP1_System systemDynamics(referenceManagerPtr);
  bolza::TimeTriggeredRollout Rollout(systemDynamics, RolloutSettings());

  // instantiate
  bolza::ILQR ddp(ddpSettings, rollout, problem, *initializerPtr);
  ddp.SetReferenceManager(referenceManagerPtr);

  if (ddpSettings.displayInfo_ || ddpSettings.displayShortSummary_) {
    std::cerr << "\n" << getTestName(ddpSettings) << "\n";
  }

  // run ddp
  ddp.Run(startTime, initState, finalTime);
  // get performance index
  const auto performanceIndex = ddp.GetPerformanceIndeces();

  // performanceIndeces test
  performanceIndexTest(ddpSettings, performanceIndex);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
INSTANTIATE_TEST_CASE_P(Exp1Case, Exp1,
                        testing::Combine(testing::ValuesIn({bolza::search_strategy::Type::LINE_SEARCH,
                                                            bolza::search_strategy::Type::LEVENBERG_MARQUARDT}),
                                         testing::ValuesIn({size_t(1), size_t(3)})), /* num threads */
                        [](const testing::TestParamInfo<Exp1::ParamType>& info) {
                          /* returns test name for gtest summary */
                          std::string name;
                          name += bolza::search_strategy::ToString(std::get<0>(info.param)) + "__";
                          name += std::get<1>(info.param) == 1 ? "SINGLE_THREAD" : "MULTI_THREAD";
                          return name;
                        });
