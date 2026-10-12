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

#include <cstdlib>
#include <ctime>
#include <iostream>
#include <memory>

#include <gtest/gtest.h>

#include <bolza/core/types.hpp>
#include <bolza/core/control/linear_controller.hpp>
#include <bolza/core/dynamics/linear_system_dynamics.hpp>
#include <bolza/oc/rollout/time_triggered_rollout.hpp>

using namespace bolza;

TEST(time_rollout_test, time_rollout_test) {
  constexpr size_t nx = 2;
  constexpr size_t nu = 1;
  const scalar_t initTime = 0.0;
  const scalar_t finalTime = 10.0;
  const vector_t initState = vector_t::Zero(nx);

  // ModeSchedule
  ModeSchedule ModeSchedule({3.0, 4.0, 4.0}, {0, 1, 2, 3});

  const matrix_t A = (matrix_t(nx, nx) << -2.0, -1.0, 1.0, 0.0).finished();
  const matrix_t B = (matrix_t(nx, nu) << 1.0, 0.0).finished();
  LinearSystemDynamics systemDynamics(A, B);

  // controller
  const scalar_array_t cntTimeStamp{initTime, finalTime};
  const vector_array_t uff(2, vector_t::Ones(nu));
  const matrix_array_t k(2, matrix_t::Zero(nu, nx));
  LinearController Controller(cntTimeStamp, uff, k);

  // Rollout Settings
  const auto rolloutSettings = [&] {
    rollout::Settings settings;
    settings.absTolODE = 1e-7;
    settings.relTolODE = 1e-5;
    settings.timeStep = 1e-3;
    settings.maxNumStepsPerSecond = 10000;
    return settings;
  }();

  // rollout class
  auto rolloutPtr = std::make_unique<TimeTriggeredRollout>(systemDynamics, rolloutSettings);

  scalar_array_t timeTrajectory;
  size_array_t postEventIndices;
  vector_array_t stateTrajectory;
  vector_array_t inputTrajectory;
  rolloutPtr->Run(initTime, initState, finalTime, &controller, modeSchedule, timeTrajectory, postEventIndices, stateTrajectory, inputTrajectory);

  // check sizes
  const auto totalSize = timeTrajectory.size();
  ASSERT_EQ(totalSize, stateTrajectory.size());
  ASSERT_EQ(totalSize, inputTrajectory.size());
}
