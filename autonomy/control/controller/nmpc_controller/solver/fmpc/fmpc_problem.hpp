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
 * @file fmpc_problem.hpp
 * @brief Fast MPC problem with inequality constraints (nmpc_fmpc port)
 */

#pragma once

#include "autonomy/control/controller/nmpc_controller/solver/ddp/ddp_problem.hpp"

namespace autonomy {
namespace control {
namespace controller {
namespace nmpc_controller {
namespace fmpc {
/**
 * @class nmpc_controller::fmpc::FmpcProblem
 * @brief FMPC problem extending DdpProblem with inequality constraints
 *
 * @tparam StateDim state dimension (fixed only)
 * @tparam InputDim input dimension (fixed or Eigen::Dynamic)
 * @tparam IneqDim inequality dimension (fixed or Eigen::Dynamic)
 */
template<int StateDim, int InputDim, int IneqDim>
class FmpcProblem : public ddp::DdpProblem<StateDim, InputDim>
{
public:
  // Type of vector of state dimension.
  using StateDimVector = typename ddp::DdpProblem<StateDim, InputDim>::StateDimVector;

  // Type of vector of input dimension.
  using InputDimVector = typename ddp::DdpProblem<StateDim, InputDim>::InputDimVector;

  // Type of vector of inequality dimension.
  using IneqDimVector = Eigen::Matrix<double, IneqDim, 1>;

  // Type of matrix of state x state dimension.
  using StateStateDimMatrix = typename ddp::DdpProblem<StateDim, InputDim>::StateStateDimMatrix;

  // Type of matrix of input x input dimension.
  using InputInputDimMatrix = typename ddp::DdpProblem<StateDim, InputDim>::InputInputDimMatrix;

  // Type of matrix of state x input dimension.
  using StateInputDimMatrix = typename ddp::DdpProblem<StateDim, InputDim>::StateInputDimMatrix;

  // Type of matrix of input x state dimension.
  using InputStateDimMatrix = typename ddp::DdpProblem<StateDim, InputDim>::InputStateDimMatrix;

  // Type of matrix of inequality x state dimension.
  using IneqStateDimMatrix = Eigen::Matrix<double, IneqDim, StateDim>;

  // Type of matrix of inequality x input dimension.
  using IneqInputDimMatrix = Eigen::Matrix<double, IneqDim, InputDim>;

public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  /**
   * @brief Constructor.
   * @param dt discretization timestep [sec]
   */
  FmpcProblem(double dt) : ddp::DdpProblem<StateDim, InputDim>(dt)
  {
    // Check dimension
    static_assert(IneqDim >= 0 || IneqDim == Eigen::Dynamic,
                  "[Fmpc] Template param IneqDim should be non-negative or Eigen::Dynamic.");
  }

  /**
   * @brief Gets the inequality dimension.
   * @note If inequality dimension is dynamic, this must not be called. Instead, ineqDim(t) must be called passing time
   * as a parameter.
   */
  inline virtual int ineqDim() const
  {
    if constexpr(IneqDim == Eigen::Dynamic)
    {
      throw std::runtime_error("Since ineq dimension is dynamic, time must be passed to ineqDim().");
    }
    return IneqDim;
  }

  /**
   * @brief Gets the inequality dimension.
   * @param t time
   * @note If inequality dimension is dynamic, this must be overridden.
   */
  inline virtual int ineqDim(double // t
  ) const
  {
    if constexpr(IneqDim == Eigen::Dynamic)
    {
      throw std::runtime_error("ineqDim(t) must be overridden if ineq dimension is dynamic.");
    }
    else
    {
      return ineqDim();
    }
  }

  /**
   * @brief Calculate inequality constraints.
   * @param t time [sec]
   * @param x current state
   * @param u current input
   * @return inequality constraints that must be less than or equal to zero
   */
  virtual IneqDimVector ineqConst(double t, const StateDimVector & x, const InputDimVector & u) const = 0;

  /**
   * @brief Calculate first-order derivatives of inequality constraints.
   * @param t time [sec]
   * @param x state
   * @param u input
   * @param ineq_const_deriv_x first-order derivative of inequality constraints w.r.t. state
   * @param ineq_const_deriv_u first-order derivative of inequality constraints w.r.t. input
   */
  virtual void calcIneqConstDeriv(double t,
                                  const StateDimVector & x,
                                  const InputDimVector & u,
                                  Eigen::Ref<IneqStateDimMatrix> ineq_const_deriv_x,
                                  Eigen::Ref<IneqInputDimMatrix> ineq_const_deriv_u) const = 0;

  using ddp::DdpProblem<StateDim, InputDim>::calcStateEqDeriv;

private:
  /**
   * @brief Calculate first-order and second-order derivatives of discrete state equation.
   * @param t time [sec]
   * @param x state
   * @param u input
   * @param state_eq_deriv_x first-order derivative of state equation w.r.t. state
   * @param state_eq_deriv_u first-order derivative of state equation w.r.t. input
   * @param state_eq_deriv_xx second-order derivative of state equation w.r.t. state
   * @param state_eq_deriv_uu second-order derivative of state equation w.r.t. input
   * @param state_eq_deriv_xu second-order derivative of state equation w.r.t. state and input
   */
  inline virtual void calcStateEqDeriv(double, // t
                                       const StateDimVector &, // x
                                       const InputDimVector &, // u
                                       Eigen::Ref<StateStateDimMatrix>, // state_eq_deriv_x
                                       Eigen::Ref<StateInputDimMatrix>, // state_eq_deriv_u
                                       std::vector<StateStateDimMatrix> &, // state_eq_deriv_xx
                                       std::vector<InputInputDimMatrix> &, // state_eq_deriv_uu
                                       std::vector<StateInputDimMatrix> & // state_eq_deriv_xu
  ) const override
  {
    throw std::runtime_error("[Fmpc] Second-order derivatives of state equation is not used.");
  }
};
}  // namespace fmpc
}  // namespace nmpc_controller
}  // namespace controller
}  // namespace control
}  // namespace autonomy
