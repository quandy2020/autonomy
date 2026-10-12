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
#include <string>

#include <filesystem>

#include <bolza/core/initialization/default_initializer.hpp>
#include <bolza/ddp/ilqr.hpp>
#include <bolza/ddp/slq.hpp>
#include <bolza/oc/oc_problem/optimal_control_problem.hpp>
#include <bolza/oc/rollout/time_triggered_rollout.hpp>
#include "helpers/circular_kinematics.hpp"

class CircularKinematicsTest : public testing::TestWithParam<std::tuple<bolza::search_strategy::Type, size_t>> {
 protected:
  static constexpr size_t STATE_DIM = 2;
  static constexpr size_t INPUT_DIM = 2;
  static constexpr bolza::scalar_t timeStep = 0.01;
  static constexpr bolza::scalar_t minRelCost = 1e-3;
  static constexpr bolza::scalar_t constraintTolerance = 1e-5;
  static constexpr bolza::scalar_t expectedCost = 0.1;

  CircularKinematicsTest() {
    // optimal control problem
    std::filesystem::path filePath(__FILE__);
    problem = bolza::CreateCircularKinematicsProblem("/tmp/bolza/ddp_test_generated");

    // initializer
    initializerPtr.reset(new bolza::DefaultInitializer(INPUT_DIM));
  }

  bolza::search_strategy::Type getSearchStrategy() { return std::get<0>(GetParam()); }

  size_t getNumThreads() { return std::get<1>(GetParam()); }

  bolza::Rollout::Settings RolloutSettings(bolza::ddp::Algorithm algorithmType) const {
    bolza::Rollout::Settings rolloutSettings;
    rolloutSettings.absTolODE = 1e-9;
    rolloutSettings.relTolODE = 1e-7;
    rolloutSettings.timeStep = timeStep;
    rolloutSettings.maxNumStepsPerSecond = 10000;

    switch (algorithmType) {
      case bolza::ddp::Algorithm::SLQ:
        rolloutSettings.integratorType = bolza::IntegratorType::ODE45;
        break;
      case bolza::ddp::Algorithm::ILQR:
        rolloutSettings.integratorType = bolza::IntegratorType::RK4;
        break;
      default:
        throw std::runtime_error("Not supported Algorithm!");
    }

    return rolloutSettings;
  };

  bolza::ddp::Settings getSettings(bolza::ddp::Algorithm algorithmType, size_t numThreads, bolza::search_strategy::Type strategy,
                                  bool display = false) const {
    bolza::ddp::Settings ddpSettings;
    ddpSettings.algorithm_ = algorithmType;
    ddpSettings.nThreads_ = numThreads;
    ddpSettings.displayInfo_ = false;
    ddpSettings.displayShortSummary_ = display;
    ddpSettings.checkNumericalStability_ = false;
    ddpSettings.debugPrintRollout_ = false;
    ddpSettings.absTolODE_ = 1e-9;
    ddpSettings.relTolODE_ = 1e-7;
    ddpSettings.maxNumStepsPerSecond_ = 10000;
    ddpSettings.timeStep_ = timeStep;
    switch (algorithmType) {
      case bolza::ddp::Algorithm::SLQ:
        ddpSettings.backwardPassIntegratorType_ = bolza::IntegratorType::ODE45;
        break;
      case bolza::ddp::Algorithm::ILQR:
        ddpSettings.backwardPassIntegratorType_ = bolza::IntegratorType::RK4;
        break;
      default:
        throw std::runtime_error("Not supported Algorithm!");
    }
    ddpSettings.maxNumIterations_ = 150;
    ddpSettings.minRelCost_ = minRelCost;
    ddpSettings.constraintTolerance_ = constraintTolerance;
    ddpSettings.constraintPenaltyInitialValue_ = 2.0;
    ddpSettings.constraintPenaltyIncreaseRate_ = 1.5;
    ddpSettings.preComputeRiccatiTerms_ = false;
    ddpSettings.strategy_ = strategy;
    ddpSettings.lineSearch_.minStepLength = 0.01;
    ddpSettings.lineSearch_.hessianCorrectionStrategy = bolza::hessian_correction::Strategy::CHOLESKY_MODIFICATION;
    ddpSettings.lineSearch_.hessianCorrectionMultiple = 1e-3;
    return ddpSettings;
  }

  std::string getTestName(const bolza::ddp::Settings& ddpSettings) const {
    std::string testName;
    testName += "Circular-Kinematics Test { ";
    testName += "Algorithm: " + bolza::ddp::ToAlgorithmName(ddpSettings.algorithm_) + ",  ";
    testName += "Strategy: " + bolza::search_strategy::ToString(ddpSettings.strategy_) + ",  ";
    testName += "#threads: " + std::to_string(ddpSettings.nThreads_) + " }";
    return testName;
  }

  void performanceIndexTest(const bolza::ddp::Settings& ddpSettings, const bolza::PerformanceIndex& performanceIndex) const {
    const auto testName = getTestName(ddpSettings);
    EXPECT_LT(performanceIndex.cost - expectedCost, 0.0) << "MESSAGE: " << testName << ": failed in the cost test!";
    EXPECT_NEAR(performanceIndex.equalityConstraintsSSE, 0.0, 10.0 * constraintTolerance)
        << "MESSAGE: " << testName << ": failed in state-input equality constraint ISE test!";
  }

  const bolza::scalar_t startTime = 0.0;
  const bolza::scalar_t finalTime = 10.0;
  const bolza::vector_t initState = (bolza::vector_t(STATE_DIM) << 1.0, 0.0).finished();  // radius 1.0

  bolza::OptimalControlProblem problem;
  std::unique_ptr<bolza::Initializer> initializerPtr;
};

constexpr size_t CircularKinematicsTest::STATE_DIM;
constexpr size_t CircularKinematicsTest::INPUT_DIM;
constexpr bolza::scalar_t CircularKinematicsTest::timeStep;
constexpr bolza::scalar_t CircularKinematicsTest::minRelCost;
constexpr bolza::scalar_t CircularKinematicsTest::constraintTolerance;
constexpr bolza::scalar_t CircularKinematicsTest::expectedCost;

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
TEST_P(CircularKinematicsTest, SLQ) {
  const auto algorithm = bolza::ddp::Algorithm::SLQ;

  // ddp settings
  const auto ddpSettings = getSettings(algorithm, getNumThreads(), getSearchStrategy());

  // dynamics and rollout
  const bolza::CircularKinematicsSystem systemDynamics;
  const bolza::TimeTriggeredRollout Rollout(systemDynamics, RolloutSettings(algorithm));

  // instantiate
  bolza::SLQ ddp(ddpSettings, rollout, problem, *initializerPtr);

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
TEST_P(CircularKinematicsTest, ILQR) {
  const auto algorithm = bolza::ddp::Algorithm::ILQR;

  // ddp settings
  const auto ddpSettings = getSettings(algorithm, getNumThreads(), getSearchStrategy());

  // dynamics and rollout
  const bolza::CircularKinematicsSystem systemDynamics;
  const bolza::TimeTriggeredRollout Rollout(systemDynamics, RolloutSettings(algorithm));

  // instantiate
  bolza::ILQR ddp(ddpSettings, rollout, problem, *initializerPtr);

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
INSTANTIATE_TEST_CASE_P(CircularKinematicsTestCase, CircularKinematicsTest,
                        testing::Combine(testing::ValuesIn({bolza::search_strategy::Type::LINE_SEARCH
                                                            /* , bolza::search_strategy::Type::LEVENBERG_MARQUARDT */}),
                                         testing::ValuesIn({size_t(1), size_t(3)})), /* num threads */
                        [](const testing::TestParamInfo<CircularKinematicsTest::ParamType>& info) {
                          /* returns test name for gtest summary */
                          std::string name;
                          name += bolza::search_strategy::ToString(std::get<0>(info.param)) + "__";
                          name += std::get<1>(info.param) == 1 ? "SINGLE_THREAD" : "MULTI_THREAD";
                          return name;
                        });
