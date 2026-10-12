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

#include <bolza/core/control/linear_controller.hpp>
#include <bolza/core/dynamics/linear_system_dynamics.hpp>
#include <bolza/core/integration/integrator.hpp>

TEST(RungeKuttaDormandPrince5Test, instantiate) {
  auto ode45 = bolza::NewIntegrator(bolza::IntegratorType::ODE45_DP5);
}

class LinearSystem final : public bolza::OdeBase {
 public:
  ~LinearSystem() override = default;
  bolza::vector_t ComputeFlowMap(bolza::scalar_t t, const bolza::vector_t& x) override {
    const bolza::matrix_t A = (bolza::matrix_t(2, 2) << -2, -1,  // clang-format off
                                                       1,  0).finished();  // clang-format on
    const bolza::vector_t B = (bolza::vector_t(2) << 1, 0).finished();
    const bolza::vector_t u = bolza::vector_t::Ones(1);
    return A * x + B * u;
  }
};

TEST(RungeKuttaDormandPrince5Test, integrateLinearSystem) {
  bolza::scalar_array_t timeTrajectory;
  bolza::vector_array_t stateTrajectory;

  LinearSystem sys;
  auto integrator = bolza::NewIntegrator(bolza::IntegratorType::ODE45_DP5);

  const bolza::scalar_t t0 = 0.0;
  const bolza::scalar_t t1 = 10.0;
  const bolza::scalar_t dt = 0.01;
  bolza::vector_t x0 = bolza::vector_t::Zero(2);
  bolza::Observer Observer(&stateTrajectory, &timeTrajectory);
  integrator->IntegrateAdaptive(sys, observer, x0, t0, t1, dt);

  EXPECT_NEAR(timeTrajectory.front(), t0, 1e-6);
  EXPECT_NEAR(timeTrajectory.back(), t1, 1e-6);
  EXPECT_TRUE(stateTrajectory.front().isApprox(x0));
  EXPECT_NEAR(stateTrajectory.back()(0), 0.0, 1e-3);
  EXPECT_NEAR(stateTrajectory.back()(1), 1.0, 1e-3);
}

TEST(RungeKuttaDormandPrince5Test, IntegrateAdaptiveCompareWithODE45) {
  const bolza::scalar_t t0 = 0.0;
  const bolza::scalar_t t1 = 10.0;
  const bolza::scalar_t dt = 0.05;
  const bolza::vector_t x0 = bolza::vector_t::Zero(2);

  LinearSystem sys;

  bolza::scalar_array_t tTraj;
  bolza::vector_array_t xTraj;
  bolza::Observer Observer(&xTraj, &tTraj);
  auto integrator = bolza::NewIntegrator(bolza::IntegratorType::ODE45_DP5);
  integrator->IntegrateAdaptive(sys, observer, x0, t0, t1, dt);

  bolza::scalar_array_t tTrajOde45;
  bolza::vector_array_t xTrajOde45;
  bolza::Observer observerOde45(&xTrajOde45, &tTrajOde45);
  auto integratorOde45 = bolza::NewIntegrator(bolza::IntegratorType::ODE45);
  integratorOde45->IntegrateAdaptive(sys, observerOde45, x0, t0, t1, dt);

  for (size_t i = 0; i < tTraj.size(); i++) {
    EXPECT_NEAR(tTraj[i], tTrajOde45[i], 1e-6);
    EXPECT_TRUE(xTraj[i].isApprox(xTrajOde45[i], 1e-6));
  }
}

TEST(RungeKuttaDormandPrince5Test, IntegrateTimesCompareWithODE45) {
  const bolza::scalar_t t0 = 0.0;
  const bolza::scalar_t t1 = 10.0;
  const bolza::scalar_t dt = 0.05;
  const bolza::vector_t x0 = bolza::vector_t::Zero(2);

  LinearSystem sys;

  bolza::scalar_array_t times = {0.0, 2.0, 4.0, 6.0, 8.0, 10.0};

  bolza::vector_array_t xTraj;
  bolza::Observer Observer(&xTraj);
  auto integrator = bolza::NewIntegrator(bolza::IntegratorType::ODE45_DP5);
  integrator->IntegrateTimes(sys, observer, x0, times.begin(), times.end(), dt);

  bolza::vector_array_t xTrajOde45;
  bolza::Observer observerOde45(&xTrajOde45);
  auto integratorOde45 = bolza::NewIntegrator(bolza::IntegratorType::ODE45);
  integratorOde45->IntegrateTimes(sys, observerOde45, x0, times.begin(), times.end(), dt);

  for (size_t i = 0; i < times.size(); i++) {
    EXPECT_TRUE(xTraj[i].isApprox(xTrajOde45[i], 1e-6));
  }
}

TEST(RungeKuttaDormandPrince5Test, integrateBackwards) {
  LinearSystem sys;
  auto integrator = bolza::NewIntegrator(bolza::IntegratorType::ODE45_DP5);

  const bolza::scalar_t t0 = 0.0;
  const bolza::scalar_t t1 = 10.0;
  const bolza::scalar_t dt = 0.01;
  const bolza::scalar_t absTol = 1e-9;
  const bolza::scalar_t relTol = 1e-6;
  const bolza::vector_t x0 = bolza::vector_t::Zero(2);

  bolza::scalar_array_t timeTrajectory;
  bolza::vector_array_t stateTrajectory;
  auto observer = bolza::Observer(&stateTrajectory, &timeTrajectory);

  // integrate forward from t0 to t1
  integrator->IntegrateAdaptive(sys, observer, x0, t0, t1, dt, absTol, relTol);

  const bolza::vector_t x1 = stateTrajectory.back();
  stateTrajectory.clear();
  timeTrajectory.clear();
  observer = bolza::Observer(&stateTrajectory, &timeTrajectory);

  // integrate backward from t1 to t0
  integrator->IntegrateAdaptive(sys, observer, x1, t1, t0, -dt, absTol, relTol);

  EXPECT_NEAR(timeTrajectory.front(), t1, 1e-6);
  EXPECT_NEAR(timeTrajectory.back(), t0, 1e-6);
  EXPECT_TRUE(stateTrajectory.front().isApprox(x1));
  // Choosing an appropriate tolerance is tricky
  EXPECT_TRUE((stateTrajectory.back() - x0).norm() < 1e-3);
}
