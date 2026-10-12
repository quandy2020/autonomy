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

#include "bolza/core/integration/integrator.hpp"
#include "bolza/core/integration/sensitivity_integrator.hpp"

#include <bolza/core/control/feedforward_controller.hpp>
#include <bolza/core/dynamics/linear_system_dynamics.hpp>
#include <bolza/core/dynamics/system_dynamics_base.hpp>

namespace {
std::unique_ptr<bolza::LinearSystemDynamics> getSystem() {
  bolza::matrix_t A(2, 2);
  A << -2, -1,  // clang-format off
        1,  0;  // clang-format on
  bolza::matrix_t B(2, 1);
  B << 1, 0;
  return std::make_unique<bolza::LinearSystemDynamics>(std::move(A), std::move(B));
}
}  // namespace

TEST(test_sensitivity_integrator, eulerSensitivity) {
  auto type = bolza::SensitivityIntegratorType::EULER;
  auto eulerSensitivityDiscretization = bolza::SelectDynamicsSensitivityDiscretization(type);
  auto eulerDiscretization = bolza::SelectDynamicsDiscretization(type);

  auto system = getSystem();
  bolza::scalar_t t = 0.5;
  bolza::vector_t x = bolza::vector_t::Random(2);
  bolza::vector_t u = bolza::vector_t::Random(1);
  bolza::scalar_t dt = 0.1;

  // Check with more readable version of sensitivity computation.
  const auto eulerdynamics_check = [&]() {
    const bolza::PreComputation preComp;

    // System evaluations
    const bolza::VectorFunctionLinearApproximation k1 = system->LinearApproximation(t, x, u, preComp);

    // State sensitivity \dot{Sx} = dfdx(t) Sx, with Sx(0) = Identity()
    const bolza::matrix_t dk1dxk = k1.dfdx;

    // Input sensitivity \dot{Su} = dfdx(t) Su + dfdu(t), with Su(0) = Zero()
    const bolza::matrix_t dk1duk = k1.dfdu;

    // Assemble discrete approximation
    bolza::VectorFunctionLinearApproximation discreteApproximation;
    discreteApproximation.dfdx = bolza::matrix_t::Identity(x.size(), x.size()) + dt * dk1dxk;
    discreteApproximation.dfdu = dt * dk1duk;
    discreteApproximation.f = x + dt * k1.f;
    return discreteApproximation;
  }();

  const auto eulerForwardDynamics = EulerDiscretization(*system, t, x, u, dt);
  ASSERT_TRUE(eulerForwardDynamics.isApprox(eulerdynamics_check.f));
  const auto eulerLinearizedDynamics = EulerSensitivityDiscretization(*system, t, x, u, dt);
  ASSERT_TRUE(eulerLinearizedDynamics.f.isApprox(eulerdynamics_check.f));
  ASSERT_TRUE(eulerLinearizedDynamics.dfdx.isApprox(eulerdynamics_check.dfdx));
  ASSERT_TRUE(eulerLinearizedDynamics.dfdu.isApprox(eulerdynamics_check.dfdu));
}

TEST(test_sensitivity_integrator, rk2Sensitivity) {
  auto type = bolza::SensitivityIntegratorType::RK2;
  auto rk2SensitivityDiscretization = bolza::SelectDynamicsSensitivityDiscretization(type);
  auto rk2Discretization = bolza::SelectDynamicsDiscretization(type);

  auto system = getSystem();
  bolza::scalar_t t = 0.5;
  bolza::vector_t x = bolza::vector_t::Random(2);
  bolza::vector_t u = bolza::vector_t::Random(1);
  bolza::scalar_t dt = 0.1;

  // Check with more readable version of sensitivity computation.
  const auto rk2dynamics_check = [&]() {
    const bolza::scalar_t dt_halve = dt / 2.0;
    const bolza::PreComputation preComp;

    // System evaluations
    const bolza::VectorFunctionLinearApproximation k1 = system->LinearApproximation(t, x, u, preComp);
    const bolza::VectorFunctionLinearApproximation k2 = system->LinearApproximation(t + dt, x + dt * k1.f, u, preComp);

    // State sensitivity \dot{Sx} = dfdx(t) Sx, with Sx(0) = Identity()
    const bolza::matrix_t dk1dxk = k1.dfdx;
    const bolza::matrix_t dk2dxk = dt * k2.dfdx * dk1dxk + k2.dfdx;

    // Input sensitivity \dot{Su} = dfdx(t) Su + dfdu(t), with Su(0) = Zero()
    const bolza::matrix_t dk1duk = k1.dfdu;
    const bolza::matrix_t dk2duk = dt * k2.dfdx * dk1duk + k2.dfdu;

    // Assemble discrete approximation
    bolza::VectorFunctionLinearApproximation discreteApproximation;
    discreteApproximation.dfdx = bolza::matrix_t::Identity(x.size(), x.size()) + dt_halve * dk1dxk + dt_halve * dk2dxk;
    discreteApproximation.dfdu = dt_halve * dk1duk + dt_halve * dk2duk;
    discreteApproximation.f = x + dt_halve * k1.f + dt_halve * k2.f;
    return discreteApproximation;
  }();

  const auto rk2ForwardDynamics = Rk2Discretization(*system, t, x, u, dt);
  ASSERT_TRUE(rk2ForwardDynamics.isApprox(rk2dynamics_check.f));
  const auto rk2LinearizedDynamics = Rk2SensitivityDiscretization(*system, t, x, u, dt);
  ASSERT_TRUE(rk2LinearizedDynamics.f.isApprox(rk2dynamics_check.f));
  ASSERT_TRUE(rk2LinearizedDynamics.dfdx.isApprox(rk2dynamics_check.dfdx));
  ASSERT_TRUE(rk2LinearizedDynamics.dfdu.isApprox(rk2dynamics_check.dfdu));
}

TEST(test_sensitivity_integrator, rk4Sensitivity) {
  auto type = bolza::SensitivityIntegratorType::RK4;
  auto rk4SensitivityDiscretization = bolza::SelectDynamicsSensitivityDiscretization(type);
  auto rk4Discretization = bolza::SelectDynamicsDiscretization(type);

  auto system = getSystem();
  bolza::scalar_t t = 0.5;
  bolza::vector_t x = bolza::vector_t::Random(2);
  bolza::vector_t u = bolza::vector_t::Random(1);
  bolza::scalar_t dt = 0.1;

  // Check with more readable version of sensitivity computation.
  const auto rk4dynamics_check = [&]() {
    const bolza::scalar_t dt_halve = dt / 2.0;
    const bolza::scalar_t dt_sixth = dt / 6.0;
    const bolza::scalar_t dt_third = dt / 3.0;
    const bolza::PreComputation preComp;

    // System evaluations
    const bolza::VectorFunctionLinearApproximation k1 = system->LinearApproximation(t, x, u, preComp);
    const bolza::VectorFunctionLinearApproximation k2 = system->LinearApproximation(t + dt_halve, x + dt_halve * k1.f, u, preComp);
    const bolza::VectorFunctionLinearApproximation k3 = system->LinearApproximation(t + dt_halve, x + dt_halve * k2.f, u, preComp);
    const bolza::VectorFunctionLinearApproximation k4 = system->LinearApproximation(t + dt, x + dt * k3.f, u, preComp);

    // State sensitivity \dot{Sx} = dfdx(t) Sx, with Sx(0) = Identity()
    const bolza::matrix_t dk1dxk = k1.dfdx;
    const bolza::matrix_t dk2dxk = dt_halve * k2.dfdx * dk1dxk + k2.dfdx;
    const bolza::matrix_t dk3dxk = dt_halve * k3.dfdx * dk2dxk + k3.dfdx;
    const bolza::matrix_t dk4dxk = dt * k4.dfdx * dk3dxk + k4.dfdx;

    // Input sensitivity \dot{Su} = dfdx(t) Su + dfdu(t), with Su(0) = Zero()
    const bolza::matrix_t dk1duk = k1.dfdu;
    const bolza::matrix_t dk2duk = dt_halve * k2.dfdx * dk1duk + k2.dfdu;
    const bolza::matrix_t dk3duk = dt_halve * k3.dfdx * dk2duk + k3.dfdu;
    const bolza::matrix_t dk4duk = dt * k4.dfdx * dk3duk + k4.dfdu;

    // Assemble discrete approximation
    bolza::VectorFunctionLinearApproximation discreteApproximation;
    discreteApproximation.dfdx =
        bolza::matrix_t::Identity(x.size(), x.size()) + dt_sixth * dk1dxk + dt_third * dk2dxk + dt_third * dk3dxk + dt_sixth * dk4dxk;
    discreteApproximation.dfdu = dt_sixth * dk1duk + dt_third * dk2duk + dt_third * dk3duk + dt_sixth * dk4duk;
    discreteApproximation.f = x + dt_sixth * k1.f + dt_third * k2.f + dt_third * k3.f + dt_sixth * k4.f;
    return discreteApproximation;
  }();

  const auto rk4ForwardDynamics = Rk4Discretization(*system, t, x, u, dt);
  ASSERT_TRUE(rk4ForwardDynamics.isApprox(rk4dynamics_check.f));
  const auto rk4LinearizedDynamics = Rk4SensitivityDiscretization(*system, t, x, u, dt);
  ASSERT_TRUE(rk4LinearizedDynamics.f.isApprox(rk4dynamics_check.f));
  ASSERT_TRUE(rk4LinearizedDynamics.dfdx.isApprox(rk4dynamics_check.dfdx));
  ASSERT_TRUE(rk4LinearizedDynamics.dfdu.isApprox(rk4dynamics_check.dfdu));
}

TEST(test_sensitivity_integrator, vsIntegratorRK4) {
  auto system = getSystem();
  bolza::scalar_t t = 0.5;
  bolza::vector_t x = bolza::vector_t::Random(2);
  bolza::vector_t u = bolza::vector_t::Random(1);
  bolza::scalar_t dt = 0.1;

  auto integrator = NewIntegrator(bolza::IntegratorType::RK4);
  bolza::scalar_array_t timeTrajectory;
  bolza::vector_array_t stateTrajectory;
  auto observer = bolza::Observer(&stateTrajectory, &timeTrajectory);
  bolza::FeedforwardController Controller({t, t}, {u, u});
  system->SetController(&controller);
  int maxNumSteps = 4;
  integrator->IntegrateConst(*system, observer, x, t, t + dt, dt, maxNumSteps);
  const auto integratorRk4ForwardDynamics = stateTrajectory.back();

  auto type = bolza::SensitivityIntegratorType::RK4;
  auto rk4Discretization = bolza::SelectDynamicsDiscretization(type);
  const auto rk4ForwardDynamics = Rk4Discretization(*system, t, x, u, dt);

  ASSERT_TRUE(rk4ForwardDynamics.isApprox(integratorRk4ForwardDynamics));
}
