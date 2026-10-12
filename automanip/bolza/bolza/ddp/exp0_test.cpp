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
#include "helpers/exp0.hpp"

#include <bolza/ddp/ilqr.hpp>
#include <bolza/ddp/slq.hpp>

class Exp0 : public testing::Test {
 protected:
  static constexpr size_t STATE_DIM = 2;
  static constexpr size_t INPUT_DIM = 1;
  static constexpr bolza::scalar_t timeStep = 1e-2;
  static constexpr bolza::scalar_t minRelCost = 1e-3;
  static constexpr bolza::scalar_t expectedCost = 9.766;

  Exp0() {
    // event times
    const bolza::scalar_array_t eventTimes{0.1897};
    const std::vector<size_t> modeSequence{0, 1};
    referenceManagerPtr = bolza::GetExp0ReferenceManager(eventTimes, modeSequence);

    // optimal control problem
    problem = bolza::CreateExp0Problem(referenceManagerPtr);

    // operatingTrajectories
    initializerPtr.reset(new bolza::DefaultInitializer(INPUT_DIM));
  }

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
    ddpSettings.preComputeRiccatiTerms_ = true;
    ddpSettings.displayInfo_ = false;
    ddpSettings.displayShortSummary_ = display;
    ddpSettings.absTolODE_ = 1e-10;
    ddpSettings.relTolODE_ = 1e-7;
    ddpSettings.timeStep_ = timeStep;
    ddpSettings.backwardPassIntegratorType_ = bolza::IntegratorType::ODE45;
    ddpSettings.maxNumStepsPerSecond_ = 10000;
    ddpSettings.maxNumIterations_ = 30;
    ddpSettings.minRelCost_ = minRelCost;
    ddpSettings.checkNumericalStability_ = true;
    ddpSettings.useFeedbackPolicy_ = true;
    ddpSettings.debugPrintRollout_ = false;
    ddpSettings.strategy_ = strategy;
    ddpSettings.lineSearch_.minStepLength = 0.0001;
    return ddpSettings;
  }

  std::string getTestName(const bolza::ddp::Settings& ddpSettings) const {
    std::string testName;
    testName += "EXP0 Test { ";
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
  const bolza::scalar_t finalTime = 2.0;
  const bolza::vector_t initState = (bolza::vector_t(STATE_DIM) << 0.0, 2.0).finished();
  std::shared_ptr<bolza::ReferenceManager> referenceManagerPtr;

  bolza::OptimalControlProblem problem;
  std::unique_ptr<bolza::Initializer> initializerPtr;
};

constexpr size_t Exp0::STATE_DIM;
constexpr size_t Exp0::INPUT_DIM;
constexpr bolza::scalar_t Exp0::timeStep;
constexpr bolza::scalar_t Exp0::minRelCost;
constexpr bolza::scalar_t Exp0::expectedCost;

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
TEST_F(Exp0, ddp_feedback_policy) {
  // ddp settings
  auto ddpSettings = getSettings(bolza::ddp::Algorithm::SLQ, 2, bolza::search_strategy::Type::LINE_SEARCH);
  ddpSettings.useFeedbackPolicy_ = true;

  // dynamics and rollout
  bolza::EXP0_System systemDynamics(referenceManagerPtr);
  bolza::TimeTriggeredRollout Rollout(systemDynamics, RolloutSettings());

  // instantiate
  bolza::SLQ ddp(ddpSettings, rollout, problem, *initializerPtr);
  ddp.SetReferenceManager(referenceManagerPtr);

  // run ddp
  ddp.Run(startTime, initState, finalTime);
  // get solution
  const auto solution = ddp.GetPrimalSolution(finalTime);
  const auto* ctrlPtr = dynamic_cast<bolza::LinearController*>(solution.controllerPtr_.get());

  EXPECT_TRUE(ctrlPtr != nullptr) << "MESSAGE: SLQ solution does not contain a linear feedback policy!";
  EXPECT_DOUBLE_EQ(ctrlPtr->timeStamp_.back(), finalTime) << "MESSAGE: SLQ failed in policy final time of controller!";
  EXPECT_DOUBLE_EQ(solution.timeTrajectory_.back(), finalTime) << "MESSAGE: SLQ failed in policy final time of trajectory!";
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
TEST_F(Exp0, ddp_feedforward_policy) {
  // ddp settings
  auto ddpSettings = getSettings(bolza::ddp::Algorithm::SLQ, 2, bolza::search_strategy::Type::LINE_SEARCH);
  ddpSettings.useFeedbackPolicy_ = false;

  // dynamics and rollout
  bolza::EXP0_System systemDynamics(referenceManagerPtr);
  bolza::TimeTriggeredRollout Rollout(systemDynamics, RolloutSettings());

  // instantiate
  bolza::SLQ ddp(ddpSettings, rollout, problem, *initializerPtr);
  ddp.SetReferenceManager(referenceManagerPtr);

  // run ddp
  ddp.Run(startTime, initState, finalTime);
  // get solution
  const auto solution = ddp.GetPrimalSolution(finalTime);
  const auto* ctrlPtr = dynamic_cast<bolza::FeedforwardController*>(solution.controllerPtr_.get());

  EXPECT_TRUE(ctrlPtr != nullptr) << "MESSAGE: SLQ solution does not contain a feedforward policy!";
  EXPECT_DOUBLE_EQ(ctrlPtr->timeStamp_.back(), finalTime) << "MESSAGE: SLQ failed in policy final time of controller!";
  EXPECT_DOUBLE_EQ(solution.timeTrajectory_.back(), finalTime) << "MESSAGE: SLQ failed in policy final time of trajectory!";
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
TEST_F(Exp0, ddp_moving_horizon) {
  // ddp settings
  constexpr size_t numThreads = 2;
  const auto ddpSettings = getSettings(bolza::ddp::Algorithm::SLQ, numThreads, bolza::search_strategy::Type::LINE_SEARCH);

  // event times
  const bolza::scalar_array_t eventTimes{1.0};
  const bolza::size_array_t modeSequence{0, 1};
  referenceManagerPtr = bolza::GetExp0ReferenceManager(eventTimes, modeSequence);

  // dynamics and rollout
  bolza::EXP0_System systemDynamics(referenceManagerPtr);
  bolza::TimeTriggeredRollout Rollout(systemDynamics, RolloutSettings());

  // instantiate
  bolza::SLQ ddp(ddpSettings, rollout, problem, *initializerPtr);
  ddp.SetReferenceManager(referenceManagerPtr);

  // run SLQ (no active event)
  bolza::scalar_t startTime = 0.2;
  bolza::scalar_t finalTime = 0.9;
  EXPECT_NO_THROW(ddp.Run(startTime, initState, finalTime));

  // move the time horizon (overlap with one active event)
  startTime = 0.6;
  finalTime = 1.2;
  EXPECT_NO_THROW(ddp.Run(startTime, initState, finalTime));

  // move the time horizon (overlap with no active event)
  startTime = 1.1;
  finalTime = 1.5;
  EXPECT_NO_THROW(ddp.Run(startTime, initState, finalTime));

  // move the time horizon (no overlap)
  startTime = 1.6;
  finalTime = 2.0;
  EXPECT_NO_THROW(ddp.Run(startTime, initState, finalTime));
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
TEST_F(Exp0, ddp_hamiltonian) {
  // ddp settings
  constexpr size_t numThreads = 2;
  auto ddpSettings = getSettings(bolza::ddp::Algorithm::SLQ, numThreads, bolza::search_strategy::Type::LINE_SEARCH);
  ddpSettings.useFeedbackPolicy_ = true;
  ddpSettings.maxNumIterations_ = 50;
  ddpSettings.minRelCost_ = 1e-9;  // to allow more iterations that the effect of final linesearch is negligible

  // dynamics and rollout
  bolza::EXP0_System systemDynamics(referenceManagerPtr);
  bolza::TimeTriggeredRollout Rollout(systemDynamics, RolloutSettings());

  // instantiate
  bolza::SLQ ddp(ddpSettings, rollout, problem, *initializerPtr);
  ddp.SetReferenceManager(referenceManagerPtr);

  // run ddp
  ddp.Run(startTime, initState, finalTime);
  // get solution
  const auto solution = ddp.GetPrimalSolution(finalTime);

  // define precision for tests
  constexpr bolza::scalar_t precision = 1e-3;

  // get Hamiltonian at current solution
  // expected outcome: true, because the current solution should be optimal
  bolza::scalar_t time = solution.timeTrajectory_.front();
  bolza::vector_t state = solution.stateTrajectory_.front();
  bolza::vector_t input = solution.controllerPtr_->ComputeInput(time, state);
  auto hamiltonian = ddp.GetHamiltonian(time, state, input);
  const bolza::vector_t dHdu1a = hamiltonian.dfdu;
  EXPECT_TRUE(dHdu1a.isZero(precision)) << "MESSAGE for test 1a: Derivative of Hamiltonian w.r.t. to u is not zero: " << dHdu1a.transpose();

  // evaluate Hamiltonian at different state (but using feedback policy)
  // expected outcome: true, because for a linear system the LQ approximation of H is exact and the linear feedback policy is globally
  // optimal
  bolza::scalar_t querryTime = solution.timeTrajectory_.front();
  bolza::vector_t querryState = bolza::vector_t::Random(solution.stateTrajectory_.front().size());
  bolza::vector_t querryInput = solution.controllerPtr_->ComputeInput(querryTime, querryState);
  const bolza::vector_t dHdu1b = hamiltonian.dfdux * (querryState - state) + hamiltonian.dfduu * (querryInput - input) + hamiltonian.dfdu;
  EXPECT_TRUE(dHdu1b.isZero(precision)) << "MESSAGE for test 1b: Derivative of Hamiltonian w.r.t. to u is not zero: " << dHdu1b.transpose();

  // evaluate Hamiltonian at different input
  // expected outcome: false, because for a linear system the LQ approximation of H is exact and a random input is not optimal
  querryTime = solution.timeTrajectory_.front();
  querryState = solution.stateTrajectory_.front();
  querryInput = bolza::vector_t::Random(solution.inputTrajectory_.front().size());
  const bolza::vector_t dHdu1c = hamiltonian.dfdux * (querryState - state) + hamiltonian.dfduu * (querryInput - input) + hamiltonian.dfdu;
  EXPECT_FALSE(dHdu1c.isZero(precision)) << "MESSAGE for test 1c: Derivative of Hamiltonian w.r.t. to u is zero: " << dHdu1c.transpose();

  // get Hamiltonian at different state (but using feedback policy)
  // expected outcome: true, because for a linear system the linear feedback policy is globally optimal
  time = solution.timeTrajectory_.front();
  state = bolza::vector_t::Random(solution.stateTrajectory_.front().size());
  input = solution.controllerPtr_->ComputeInput(time, state);
  hamiltonian = ddp.GetHamiltonian(time, state, input);
  const bolza::vector_t dHdu2 = hamiltonian.dfdu;
  EXPECT_TRUE(dHdu2.isZero(precision)) << "MESSAGE for test 2: Derivative of Hamiltonian w.r.t. to u is not zero: " << dHdu2.transpose();

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
/* Add parameterized test suite */
class Exp0Param : public Exp0, public testing::WithParamInterface<std::tuple<bolza::search_strategy::Type, size_t>> {
 protected:
  bolza::search_strategy::Type getSearchStrategy() { return std::get<0>(GetParam()); }

  size_t getNumThreads() { return std::get<1>(GetParam()); }
};

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
TEST_P(Exp0Param, SLQ) {
  // ddp settings
  const auto ddpSettings = getSettings(bolza::ddp::Algorithm::SLQ, getNumThreads(), getSearchStrategy());

  // dynamics and rollout
  bolza::EXP0_System systemDynamics(referenceManagerPtr);
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
TEST_P(Exp0Param, ILQR) {
  // ddp settings
  const auto ddpSettings = getSettings(bolza::ddp::Algorithm::ILQR, getNumThreads(), getSearchStrategy());

  // dynamics and rollout
  bolza::EXP0_System systemDynamics(referenceManagerPtr);
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
INSTANTIATE_TEST_CASE_P(Exp0ParamCase, Exp0Param,
                        testing::Combine(testing::ValuesIn({bolza::search_strategy::Type::LINE_SEARCH,
                                                            bolza::search_strategy::Type::LEVENBERG_MARQUARDT}),
                                         testing::ValuesIn({size_t(1), size_t(3)})), /* num threads */
                        [](const testing::TestParamInfo<Exp0Param::ParamType>& info) {
                          /* returns test name for gtest summary */
                          std::string name;
                          name += bolza::search_strategy::ToString(std::get<0>(info.param)) + "__";
                          name += std::get<1>(info.param) == 1 ? "SINGLE_THREAD" : "MULTI_THREAD";
                          return name;
                        });
