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
 * @file cgmres_solver.hpp
 * @brief Continuation/GMRES NMPC solver (nmpc_cgmres port)
 */

#pragma once

#include <fstream>
#include <iostream>
#include <memory>

#include "autonomy/control/controller/nmpc_controller/solver/cgmres/cgmres_problem.hpp"
#include "autonomy/control/controller/nmpc_controller/solver/cgmres/ode_solver.hpp"

namespace autonomy {
namespace control {
namespace controller {
namespace nmpc_controller {
namespace cgmres {
/**
 * @class nmpc_controller::cgmres::CgmresSolver
 * @brief C/GMRES solver for fast nonlinear receding-horizon control
 *
 * See Ohtsuka, Automatica 2004 (Continuation/GMRES).
 */
class CgmresSolver
{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  /**
   * @brief Construct solver with problem and ODE integrators
   * @param problem Optimal-control problem definition
   * @param ode_solver Integrator for adjoint/state rollouts inside C/GMRES
   * @param sim_ode_solver Integrator for closed-loop simulation (defaults to ode_solver)
   */
  CgmresSolver(std::shared_ptr<CgmresProblem> problem,
               std::shared_ptr<OdeSolver> ode_solver,
               std::shared_ptr<OdeSolver> sim_ode_solver = nullptr)
  : problem_(problem), ode_solver_(ode_solver), sim_ode_solver_(sim_ode_solver)
  {
    if(!sim_ode_solver_)
    {
      sim_ode_solver_ = ode_solver_;
    }
  }

  /**
   * @brief Allocate horizon buffers and reset internal state.
   */
  void setup();

  /**
   * @brief Run offline continuation/GMRES until the horizon reaches steady_horizon_duration_.
   */
  void run();

  /**
   * @brief Compute one receding-horizon control step (real-time API)
   * @param t Current time [s]
   * @param x Current measured state
   * @param next_x State one step ahead (used for finite-difference correction)
   * @param u Output control at time t
   */
  void calcControlInput(double t,
                        const Eigen::Ref<const Eigen::VectorXd> & x,
                        const Eigen::Ref<const Eigen::VectorXd> & next_x,
                        Eigen::Ref<Eigen::VectorXd> u);

  /**
   * @brief Stack \f$ \partial h / \partial u \f$ over the prediction horizon
   * @param t Current time [s]
   * @param x Current state
   * @param u_list Control sequence (horizon x dim_u)
   * @param DhDu_list Output Jacobian stack used by GMRES
   */
  void calcDhDuList(double t,
                    const Eigen::Ref<const Eigen::VectorXd> & x,
                    const Eigen::Ref<const Eigen::MatrixXd> & u_list,
                    Eigen::Ref<Eigen::MatrixXd> DhDu_list);

  /**
   * @brief Matrix-vector product with the stacked Hamiltonian Hessian (GMRES callback).
   */
  Eigen::VectorXd eqAmulFunc(const Eigen::Ref<const Eigen::VectorXd> & vec);

public:
  // Problem model
  std::shared_ptr<CgmresProblem> problem_;
  // Integrator for C/GMRES iterations
  std::shared_ptr<OdeSolver> ode_solver_;
  // Integrator for plant simulation
  std::shared_ptr<OdeSolver> sim_ode_solver_;

  // C/GMRES algorithm parameters
  // Total offline simulation horizon [s]
  double sim_duration_ = 10;
  // Target prediction horizon [s]
  double steady_horizon_duration_ = 1.0;
  // Horizon discretization count
  int horizon_divide_num_ = 25;
  // Continuation step for horizon growth
  double horizon_increase_ratio_ = 0.5;

  // Internal integration step [s]
  double dt_ = 0.001;

  // GMRES regularization / scaling
  double eq_zeta_ = 1000.0;
  // Maximum GMRES iterations per step
  int k_max_ = 5;

  // Perturbation for finite-difference terms
  double finite_diff_delta_ = 0.002;

  // Trace dump interval (0 = disabled)
  int dump_step_ = 5;

  // Working buffers updated during solve()
  // Current state vector during C/GMRES iteration
  Eigen::VectorXd x_;
  // Current stacked control vector over the horizon
  Eigen::VectorXd u_;

  // Time stamp used for finite-difference perturbation
  double t_with_delta_;
  // State at t_with_delta_ for finite-difference terms
  Eigen::VectorXd x_with_delta_;

  // State trajectory over horizon
  Eigen::MatrixXd x_list_;
  // Costate trajectory over horizon
  Eigen::MatrixXd lmd_list_;

  // Control sequence over the prediction horizon
  Eigen::MatrixXd u_list_;
  // Control sequence workspace for the Amul callback
  Eigen::MatrixXd u_list_Amul_func_;

  // Stacked d(H)/d(u) Jacobian over the horizon
  Eigen::MatrixXd DhDu_list_;
  // Perturbed DhDu stack for finite-difference evaluation
  Eigen::MatrixXd DhDu_list_with_delta_;
  // DhDu workspace for the Amul callback
  Eigen::MatrixXd DhDu_list_Amul_func_;
  // Eigen::Map does not have a default constructor
  // Map view of DhDu_list_ as a flat vector for GMRES
  std::shared_ptr<Eigen::Map<Eigen::VectorXd>> DhDu_vec_;
  // Map view of the perturbed DhDu stack
  std::shared_ptr<Eigen::Map<Eigen::VectorXd>> DhDu_vec_with_delta_;
  // Map view of DhDu_list_Amul_func_
  std::shared_ptr<Eigen::Map<Eigen::VectorXd>> DhDu_vec_Amul_func_;

  // GMRES search direction / control increment vector
  Eigen::VectorXd delta_u_vec_;

  // Optional trace outputs
  // Optional trace stream for state trajectory logging
  std::ofstream ofs_x_;
  // Optional trace stream for control trajectory logging
  std::ofstream ofs_u_;
  // Optional trace stream for GMRES residual logging
  std::ofstream ofs_err_;
  // Eigen IO format used by dumpData()
  const Eigen::IOFormat vecfmt_dump_ = Eigen::IOFormat(Eigen::StreamPrecision, 0, ", ", ", ", "", "", "", "");
};
}  // namespace cgmres
}  // namespace nmpc_controller
}  // namespace controller
}  // namespace control
}  // namespace autonomy
