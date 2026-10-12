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

#include <iostream>

#include <bolza/core/types.hpp>
#include <bolza/core/control/linear_controller.hpp>
#include <bolza/core/initialization/operating_points.hpp>
#include <bolza/ddp/slq.hpp>
#include <bolza/oc/rollout/state_triggered_rollout.hpp>

#include "helpers/bouncingmass/overall_reference.hpp"
#include "helpers/bouncingmass/system_model.hpp"

/*
 * Test for State-Triggered hybrid SLQ
 *
 * The system being tested is a system consisting of a mass and a wall
 * the mass bounces against the wall. The control objective is to follow a predefined
 * trajectory consisting of fifth order polynomial.
 *
 * The system is identical to the example presented in
 * [On Optimal Trajectory Tracking for Mechanical Systems with Unilateral Constraints
 * by M. Rijnen]
 *
 * Guard Surfaces are:    x[0] > 0
 *
 * Cost function is:      x(t)^T Q x(t) + u(t)^T R u(t) + x(t1)^T P x(t1)^T
 *                        Q = [50,   0;
 *                             0,  50];
 *                        P = [56.63, 7.07;
 *                             7.07, 8.01];
 *                        R = 1;
 *
 * Initial controller is: K = [25,10]
 *
 * The following tests are implemented and performed:
 * (1) No penetration of Guard Surfaces
 * (2) Check of cost function compared against cost calculated during trusted run of SLQ
 */
TEST(BouncingMassTest, state_triggered_hybrid_slq) {
  // Parameters
  constexpr bool outputSolution = false;
  const bolza::scalar_t startTime = 0.0;
  const bolza::scalar_t finalTime = 2.5;
  const bolza::vector_t x0 = Eigen::Matrix<bolza::scalar_t, 3, 1>(0.7, 0.5, 0);
  const bolza::vector_t u0 = bolza::vector_t::Zero(INPUT_DIM);

  const auto ddpSettings = [&]() {
    bolza::ddp::Settings s;
    s.algorithm_ = bolza::ddp::Algorithm::SLQ;
    s.displayInfo_ = false;
    s.displayShortSummary_ = true;
    s.maxNumIterations_ = 30;
    s.minRelCost_ = 1e-4;
    s.nThreads_ = 1;
    s.checkNumericalStability_ = true;
    s.absTolODE_ = 1e-10;
    s.relTolODE_ = 1e-7;
    s.maxNumStepsPerSecond_ = 10000;
    s.useFeedbackPolicy_ = true;
    s.debugPrintRollout_ = false;
    return s;
  }();

  const auto rolloutSettings = [&]() {
    bolza::Rollout::Settings s;
    s.absTolODE = 1e-10;
    s.relTolODE = 1e-7;
    s.timeStep = 1e-3;
    s.maxNumStepsPerSecond = 10000;
    s.maxSingleEventIterations = 5;
    s.useTrajectorySpreadingController = true;
    return s;
  }();

  // Generation of Reference Trajectory
  const bolza::scalar_array_t trajTimes{0, 0.2, 0.8, 1.0, 1.2, 1.8, 2.0};
  const bolza::vector_t state0 = (bolza::vector_t(STATE_DIM) << 0.5, 0, 0).finished();  // Initial and final state
  const bolza::vector_t state1 = (bolza::vector_t(STATE_DIM) << 0, -5, 0).finished();   // Hard impact
  const bolza::vector_t state2 = (bolza::vector_t(STATE_DIM) << 0, -1, 0).finished();   // Soft impact
  const bolza::vector_array_t trajStates{state0, state1, state2, state2, state1, state2, state0};

  OverallReference reference(trajTimes, trajStates);
  constexpr bolza::scalar_t delta = 0.5;
  reference.extendref(delta);

  // OCP
  bolza::OptimalControlProblem problem;

  // Dynamics, Constraints and derivative classes
  BouncingMassDynamics systemDynamics;
  problem.dynamicsPtr.Reset(systemDynamics.Clone());

  // Cost Function
  const bolza::matrix_t Q = (bolza::matrix_t(STATE_DIM, STATE_DIM) << 50.0, 0.0, 0.0, 0.0, 50.0, 0.0, 0.0, 0.0, 0.0).finished();
  const bolza::matrix_t R = (bolza::matrix_t(INPUT_DIM, INPUT_DIM) << 1.0).finished();
  const bolza::matrix_t Qf = (bolza::matrix_t(STATE_DIM, STATE_DIM) << 56.63, 7.07, 0.0, 7.07, 8.01, 0.0, 0.0, 0.0, 0.0).finished();
  problem.costPtr->Add("cost", std::make_unique<BouncingMassCost>(reference, Q, R));
  problem.preJumpCostPtr->Add("preJumpCost", std::make_unique<BouncingMassFinalCost>(reference, Qf, finalTime));
  problem.finalCostPtr->Add("finalCost", std::make_unique<BouncingMassFinalCost>(reference, Qf, finalTime));

  // Rollout Class
  bolza::StateTriggeredRollout stateTriggeredRollout(systemDynamics, rolloutSettings);

  // Initial Controller
  const auto initController = [&]() {
    bolza::matrix_array_t controllerGainArray;
    bolza::vector_array_t controllerBiasArray;
    bolza::scalar_array_t timeStampArray;

    constexpr bolza::scalar_t controllerDeltaTime = 1e-3;  // Time step for controller time array
    constexpr bolza::scalar_t eps = bolza::numeric_traits::WeakEpsilon<bolza::scalar_t>();
    bolza::scalar_array_t controlTimes = trajTimes;
    controlTimes.push_back(finalTime);

    const bolza::matrix_t controllerGain = (bolza::matrix_t(INPUT_DIM, STATE_DIM) << 25, 10, 0).finished();
    for (int i = 0; i < controlTimes.size() - 1; i++) {
      const bolza::scalar_t timeSteps = (controlTimes[i + 1] + eps - controlTimes[i]) / controllerDeltaTime;
      bolza::scalar_t timeStamp = 0;
      for (int j = 0; j <= timeSteps; j++) {
        if (j == 0 && i < controlTimes.size() - 2) {
          timeStamp = controlTimes[i] + eps;
        } else {
          timeStamp = controlTimes[i] + j * controllerDeltaTime;
        }

        bolza::vector_t refState = reference.getState(timeStamp);
        bolza::vector_t refInput = reference.getInput(timeStamp);
        bolza::vector_t controllerBias = controllerGain * refState + refInput;

        timeStampArray.push_back(timeStamp);
        controllerGainArray.push_back(-controllerGain);
        controllerBiasArray.push_back(controllerBias);
      }
    }

    return bolza::LinearController(timeStampArray, controllerBiasArray, controllerGainArray);
  }();

  // operating point
  const bolza::OperatingPoints operatingTrajectories(x0, u0);

  // SLQ
  bolza::SLQ slq(ddpSettings, stateTriggeredRollout, problem, operatingTrajectories);
  slq.Run(startTime, x0, finalTime, &initController);
  const auto solutionST = slq.PrimalSolution(finalTime);

  // Test 1: No penetration of Guard Surfaces
  for (int i = 0; i < solutionST.stateTrajectory_.size(); i++) {
    EXPECT_GT(solutionST.stateTrajectory_[i](0), -ddpSettings.absTolODE_);

    // Display output: format: idx;time;x[0];xref[0];x[1];xref[1];u;uref
    if (outputSolution) {
      const auto idx = static_cast<int>(solutionST.stateTrajectory_[i][2]);
      const auto uRef = reference.getInput(solutionST.timeTrajectory_[i]);
      const auto xRef = reference.getState(idx, solutionST.timeTrajectory_[i]);
      std::cerr << i << ";" << idx << ";";
      std::cerr << std::setprecision(25) << solutionST.timeTrajectory_[i];
      std::cerr << std::setprecision(6) << ";" << solutionST.stateTrajectory_[i](0) << ";" << xRef[0] << ";";
      std::cerr << solutionST.stateTrajectory_[i](1) << ";" << xRef[1] << ";";
      std::cerr << solutionST.inputTrajectory_[i](0) << ";" << uRef[0] << std::endl;
    }
  }  // end of i loop

  // Test 2: Check of cost function
  constexpr bolza::scalar_t expectedCost = 7.15;
  const auto performanceIndeces = slq.GetPerformanceIndeces();
  EXPECT_LE(performanceIndeces.cost, expectedCost);
}
