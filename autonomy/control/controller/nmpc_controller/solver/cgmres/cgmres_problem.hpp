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
 * @file cgmres_problem.hpp
 * @brief C/GMRES optimal-control problem interface (nmpc_cgmres port)
 */

#pragma once

#include <fstream>
#include <iostream>

#include <Eigen/Core>
#include <Eigen/Dense>

namespace autonomy {
namespace control {
namespace controller {
namespace nmpc_controller {
namespace cgmres {
/**
 * @class nmpc_controller::cgmres::CgmresProblem
 * @brief Abstract C/GMRES problem (continuous dynamics and Hamiltonian derivatives)
 */
class CgmresProblem
{
public:
  // Type of function to return reference state.
  using RefFunc = std::function<void(double, Eigen::Ref<Eigen::VectorXd>)>;

public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  // Constructor.
  CgmresProblem() {}

  /**
   * @brief Calculate the state equation.
   */
  virtual void stateEquation(double t,
                             const Eigen::Ref<const Eigen::VectorXd> & x,
                             const Eigen::Ref<const Eigen::VectorXd> & u,
                             Eigen::Ref<Eigen::VectorXd> dotx) = 0;

  /**
   * @brief Calculate the costate equation.
   */
  virtual void costateEquation(double t,
                               const Eigen::Ref<const Eigen::VectorXd> & lmd,
                               const Eigen::Ref<const Eigen::VectorXd> & xu,
                               Eigen::Ref<Eigen::VectorXd> dotlmd) = 0;

  /**
   * @brief Calculate \f$ \frac{\partial \phi}{\partial x} \f$.
   */
  virtual void calcDphiDx(double t,
                          const Eigen::Ref<const Eigen::VectorXd> & x,
                          Eigen::Ref<Eigen::VectorXd> DphiDx) = 0;

  /**
   * @brief Calculate \f$ \frac{\partial h}{\partial u} \f$.
   */
  virtual void calcDhDu(double t,
                        const Eigen::Ref<const Eigen::VectorXd> & x,
                        const Eigen::Ref<const Eigen::VectorXd> & u,
                        const Eigen::Ref<const Eigen::VectorXd> & lmd,
                        Eigen::Ref<Eigen::VectorXd> DhDu) = 0;

  /**
   * @brief Dump model parameters.
   */
  virtual void dumpData(std::ofstream & ofs)
  {
    ofs << "\"state_eq_param\": [" << state_eq_param_.format(vecfmt_dump_) << "]," << std::endl;
  }

public:
  // State dimension
  int dim_x_;
  // Control dimension
  int dim_u_;
  // Cost dimension (running + terminal)
  int dim_c_;
  // Stacked control dimension over horizon
  int dim_uc_;

  // Serialized model parameters for logging
  Eigen::VectorXd state_eq_param_;

  // Initial state for offline run()
  Eigen::VectorXd x_initial_;
  // Initial control guess
  Eigen::VectorXd u_initial_;

  // Eigen IO format used by dumpData()
  const Eigen::IOFormat vecfmt_dump_ = Eigen::IOFormat(Eigen::StreamPrecision, 0, ", ", ", ", "", "", "", "");
};
}  // namespace cgmres
}  // namespace nmpc_controller
}  // namespace controller
}  // namespace control
}  // namespace autonomy
