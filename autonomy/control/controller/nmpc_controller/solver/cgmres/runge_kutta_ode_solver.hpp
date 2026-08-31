/*
 * Copyright 2025 The Openbot Authors (duyongquan)
 * Copyright 2022 Masaki Murooka (original NMPC implementation, BSD)
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

/**
 * @file runge_kutta_ode_solver.hpp
 * @brief Classical RK4 integrator for C/GMRES
 */

#pragma once

#include "autonomy/control/controller/nmpc_controller/solver/cgmres/ode_solver.hpp"

namespace autonomy {
namespace control {
namespace controller {
namespace nmpc_controller {
namespace cgmres {

/**
 * @class nmpc_controller::cgmres::RungeKuttaOdeSolver
 * @brief Classical RK4 ODE integrator for C/GMRES
 */
class RungeKuttaOdeSolver : public OdeSolver {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  /**
   * @brief Classical RK4 step with fixed control over the interval
   */
  void solve(const StateEquation& state_eq,
             double t,
             const Eigen::Ref<const Eigen::VectorXd>& x,
             const Eigen::Ref<const Eigen::VectorXd>& u,
             double dt,
             Eigen::Ref<Eigen::VectorXd> ret) override {
    const double dt_half = dt / 2.0;
    Eigen::VectorXd k1(x.size()), k2(x.size()), k3(x.size()), k4(x.size());
    state_eq(t, x, u, k1);
    state_eq(t + dt_half, x + dt_half * k1, u, k2);
    state_eq(t + dt_half, x + dt_half * k2, u, k3);
    state_eq(t + dt, x + dt * k3, u, k4);
    ret = x + (dt / 6.0) * (k1 + 2.0 * k2 + 2.0 * k3 + k4);
  }
};

}  // namespace cgmres
}  // namespace nmpc_controller
}  // namespace controller
}  // namespace control
}  // namespace autonomy
