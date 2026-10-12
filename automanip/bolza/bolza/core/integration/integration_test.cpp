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

#include <memory>

#include <bolza/core/control/linear_controller.hpp>
#include <bolza/core/dynamics/linear_system_dynamics.hpp>
#include <bolza/core/integration/integrator.hpp>

using namespace bolza;

std::unique_ptr<OdeBase> getSystem(LinearController& controller) {
  matrix_t A(2, 2);
  A << -2, -1,  // clang-format off
        1,  0;  // clang-format on
  matrix_t B(2, 1);
  B << 1, 0;

  auto sys = std::make_unique<LinearSystemDynamics>(std::move(A), std::move(B));
  sys->SetController(&controller);

  return std::move(sys);
}

void testSecondOrderSystem(IntegratorType integrator_type) {
  const scalar_t t0 = 0.0;
  const scalar_t t1 = 10.0;
  const scalar_t dt = 0.05;
  const vector_t x0 = vector_t::Zero(2);

  const scalar_array_t cntTimeStamp{0, 10};
  const vector_array_t uff(2, vector_t::Ones(1));
  const matrix_array_t k(2, matrix_t::Zero(1, 2));
  LinearController Controller(cntTimeStamp, uff, k);
  auto sys = getSystem(controller);

  std::unique_ptr<IntegratorBase> integrator = NewIntegrator(integrator_type);

  // Adaptive time integrator
  bolza::scalar_array_t timeTrajectory;
  bolza::vector_array_t stateTrajectory;
  auto observer = bolza::Observer(&stateTrajectory, &timeTrajectory);
  integrator->IntegrateAdaptive(*sys, observer, x0, t0, t1);

  EXPECT_NEAR(timeTrajectory.front(), t0, 1e-6);
  EXPECT_NEAR(timeTrajectory.back(), t1, 1e-6);
  EXPECT_TRUE(stateTrajectory.front().isApprox(x0, 1e-6));
  EXPECT_NEAR(stateTrajectory.back()(0), 0.0, 1e-3);
  EXPECT_NEAR(stateTrajectory.back()(1), 1.0, 1e-3);

  // Equidistant time integrator
  stateTrajectory.clear();
  timeTrajectory.clear();
  observer = bolza::Observer(&stateTrajectory, &timeTrajectory);
  integrator->IntegrateConst(*sys, observer, x0, t0, t1, dt);

  EXPECT_NEAR(timeTrajectory.front(), t0, 1e-6);
  EXPECT_NEAR(timeTrajectory.back(), t1, 1e-6);
  EXPECT_TRUE(stateTrajectory.front().isApprox(x0, 1e-6));
  EXPECT_NEAR(stateTrajectory.back()(0), 0.0, 1e-3);
  EXPECT_NEAR(stateTrajectory.back()(1), 1.0, 1e-3);

  // Integrator with given time trajectory
  stateTrajectory.clear();
  observer = bolza::Observer(&stateTrajectory);
  integrator->IntegrateTimes(*sys, observer, x0, timeTrajectory.begin(), timeTrajectory.end());

  EXPECT_NEAR(stateTrajectory.back()(0), 0.0, 1e-3);
  EXPECT_NEAR(stateTrajectory.back()(1), 1.0, 1e-3);
}

TEST(IntegrationTest, SecondOrderSystem_ODE45) {
  testSecondOrderSystem(IntegratorType::ODE45);
}

TEST(IntegrationTest, SecondOrderSystem_ODE45_DP5) {
  testSecondOrderSystem(IntegratorType::ODE45_DP5);
}

TEST(IntegrationTest, SecondOrderSystem_AdamsBashfort) {
  testSecondOrderSystem(IntegratorType::ADAMS_BASHFORTH);
}

TEST(IntegrationTest, SecondOrderSystem_AdamsBashfortMoulton) {
  testSecondOrderSystem(IntegratorType::ADAMS_BASHFORTH_MOULTON);
}

TEST(IntegrationTest, integratorType_from_string) {
  IntegratorType type = integrator_type::FromString("ODE45");
  EXPECT_EQ(type, IntegratorType::ODE45);
}

TEST(IntegrationTest, integratorType_to_string) {
  std::string name = integrator_type::ToString(IntegratorType::ODE45);
  EXPECT_EQ(name, "ODE45");
}
