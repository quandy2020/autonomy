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
 * @file ode_solver.hpp
 * @brief ODE integrator interface for C/GMRES rollouts
 */

#pragma once

#include <functional>

#include <Eigen/Core>
#include <Eigen/Dense>

namespace autonomy {
namespace control {
namespace controller {
namespace nmpc_controller {
namespace cgmres {

/**
 * @class nmpc_controller::cgmres::OdeSolver
 * @brief Virtual ODE integrator interface for C/GMRES rollouts
 */
class OdeSolver {
 public:
  using StateEquation = std::function<void(double,
                                           const Eigen::Ref<const Eigen::VectorXd>&,
                                           const Eigen::Ref<const Eigen::VectorXd>&,
                                           Eigen::Ref<Eigen::VectorXd>)>;

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  virtual ~OdeSolver() = default;

  /**
   * @brief Integrate one step of the ODE x_dot = f(t, x, u)
   * @param state_eq Continuous dynamics callback
   * @param t Current time [s]
   * @param x Current state
   * @param u Control input held over [t, t+dt]
   * @param dt Integration step [s]
   * @param ret Output state at t + dt
   */
  virtual void solve(const StateEquation& state_eq,
                     double t,
                     const Eigen::Ref<const Eigen::VectorXd>& x,
                     const Eigen::Ref<const Eigen::VectorXd>& u,
                     double dt,
                     Eigen::Ref<Eigen::VectorXd> ret) = 0;
};

}  // namespace cgmres
}  // namespace nmpc_controller
}  // namespace controller
}  // namespace control
}  // namespace autonomy
