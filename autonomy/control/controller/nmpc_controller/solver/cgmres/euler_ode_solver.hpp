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
 * @file euler_ode_solver.hpp
 * @brief Forward Euler integrator for C/GMRES
 */

#pragma once

#include "autonomy/control/controller/nmpc_controller/solver/cgmres/ode_solver.hpp"

namespace autonomy {
namespace control {
namespace controller {
namespace nmpc_controller {
namespace cgmres {

/**
 * @class nmpc_controller::cgmres::EulerOdeSolver
 * @brief Forward Euler ODE integrator for C/GMRES
 */
class EulerOdeSolver : public OdeSolver {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  /**
   * @brief Forward Euler step: x_{k+1} = x_k + dt * f(t, x_k, u)
   */
  void solve(const StateEquation& state_eq,
             double t,
             const Eigen::Ref<const Eigen::VectorXd>& x,
             const Eigen::Ref<const Eigen::VectorXd>& u,
             double dt,
             Eigen::Ref<Eigen::VectorXd> ret) override {
    Eigen::VectorXd dotx(x.size());
    state_eq(t, x, u, dotx);
    ret = x + dt * dotx;
  }
};

}  // namespace cgmres
}  // namespace nmpc_controller
}  // namespace controller
}  // namespace control
}  // namespace autonomy
