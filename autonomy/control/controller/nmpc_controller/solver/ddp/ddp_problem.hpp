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
 * @file ddp_problem.hpp
 * @brief Base optimal-control problem interface for the DDP solver (nmpc_ddp port)
 */

#pragma once

#include <vector>

#include <Eigen/Dense>

namespace autonomy {
namespace control {
namespace controller {
namespace nmpc_controller {
namespace ddp {
/**
 * @class nmpc_controller::ddp::DdpProblem
 * @brief DDP problem interface (state dynamics, costs, and derivatives)
 *
 * @tparam StateDim state dimension (fixed only)
 * @tparam InputDim input dimension (fixed or dynamic (i.e., Eigen::Dynamic))
 */
template<int StateDim, int InputDim>
class DdpProblem
{
public:
  // Type of vector of state dimension.
  using StateDimVector = Eigen::Matrix<double, StateDim, 1>;

  // Type of vector of input dimension.
  using InputDimVector = Eigen::Matrix<double, InputDim, 1>;

  // Type of matrix of state x state dimension.
  using StateStateDimMatrix = Eigen::Matrix<double, StateDim, StateDim>;

  // Type of matrix of input x input dimension.
  using InputInputDimMatrix = Eigen::Matrix<double, InputDim, InputDim>;

  // Type of matrix of state x input dimension.
  using StateInputDimMatrix = Eigen::Matrix<double, StateDim, InputDim>;

  // Type of matrix of input x state dimension.
  using InputStateDimMatrix = Eigen::Matrix<double, InputDim, StateDim>;

public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  /**
   * @brief Constructor.
   * @param dt discretization timestep [sec]
   */
  DdpProblem(double dt) : dt_(dt)
  {
    // Check dimension
    static_assert(StateDim > 0, "[Ddp] Template param StateDim should be positive.");
    static_assert(InputDim >= 0 || InputDim == Eigen::Dynamic,
                  "[Ddp] Template param InputDim should be non-negative or Eigen::Dynamic.");
  }

  /**
   * @brief Gets the state dimension.
   */
  static inline constexpr int stateDim()
  {
    return StateDim;
  }

  /**
   * @brief Gets the input dimension.
   * @note If input dimension is dynamic, this must not be called. Instead, inputDim(t) must be called passing time as
   * a parameter.
   */
  inline virtual int inputDim() const
  {
    if constexpr(InputDim == Eigen::Dynamic)
    {
      throw std::runtime_error("Since input dimension is dynamic, time must be passed to inputDim().");
    }
    return InputDim;
  }

  /**
   * @brief Gets the input dimension.
   * @param t time
   * @note If input dimension is dynamic, this must be overridden.
   */
  inline virtual int inputDim(double // t
  ) const
  {
    if constexpr(InputDim == Eigen::Dynamic)
    {
      throw std::runtime_error("inputDim(t) must be overridden if input dimension is dynamic.");
    }
    else
    {
      return inputDim();
    }
  }

  /**
   * @brief Gets the discretization timestep [sec].
   */
  inline double dt() const
  {
    return dt_;
  }

  /**
   * @brief Calculate discrete state equation.
   * @param t time [sec]
   * @param x current state (x[k])
   * @param u current input (u[k])
   * @return next state (x[k+1])
   */
  virtual StateDimVector stateEq(double t, const StateDimVector & x, const InputDimVector & u) const = 0;

  /**
   * @brief Calculate running cost.
   * @param t time [sec]
   * @param x current state (x[k])
   * @param u current input (u[k])
   * @return running cost (L[k])
   */
  virtual double runningCost(double t, const StateDimVector & x, const InputDimVector & u) const = 0;

  /**
   * @brief Calculate terminal cost.
   * @param t time [sec]
   * @param x current state (x[k])
   * @return terminal cost (phi[k])
   */
  virtual double terminalCost(double t, const StateDimVector & x) const = 0;

  /**
   * @brief Calculate first-order derivatives of discrete state equation.
   * @param t time [sec]
   * @param x state
   * @param u input
   * @param state_eq_deriv_x first-order derivative of state equation w.r.t. state
   * @param state_eq_deriv_u first-order derivative of state equation w.r.t. input
   */
  virtual void calcStateEqDeriv(double t,
                                const StateDimVector & x,
                                const InputDimVector & u,
                                Eigen::Ref<StateStateDimMatrix> state_eq_deriv_x,
                                Eigen::Ref<StateInputDimMatrix> state_eq_deriv_u) const = 0;

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
  virtual void calcStateEqDeriv(double t,
                                const StateDimVector & x,
                                const InputDimVector & u,
                                Eigen::Ref<StateStateDimMatrix> state_eq_deriv_x,
                                Eigen::Ref<StateInputDimMatrix> state_eq_deriv_u,
                                std::vector<StateStateDimMatrix> & state_eq_deriv_xx,
                                std::vector<InputInputDimMatrix> & state_eq_deriv_uu,
                                std::vector<StateInputDimMatrix> & state_eq_deriv_xu) const = 0;

  /**
   * @brief Calculate first-order derivatives of running cost.
   * @param t time [sec]
   * @param x state
   * @param u input
   * @param running_cost_deriv_x first-order derivative of running cost w.r.t. state
   * @param running_cost_deriv_u first-order derivative of running cost w.r.t. input
   */
  virtual void calcRunningCostDeriv(double t,
                                    const StateDimVector & x,
                                    const InputDimVector & u,
                                    Eigen::Ref<StateDimVector> running_cost_deriv_x,
                                    Eigen::Ref<InputDimVector> running_cost_deriv_u) const = 0;

  /**
   * @brief Calculate first-order and second-order derivatives of running cost.
   * @param t time [sec]
   * @param x state
   * @param u input
   * @param running_cost_deriv_x first-order derivative of running cost w.r.t. state
   * @param running_cost_deriv_u first-order derivative of running cost w.r.t. input
   * @param running_cost_deriv_xx second-order derivative of running cost w.r.t. state
   * @param running_cost_deriv_uu second-order derivative of running cost w.r.t. input
   * @param running_cost_deriv_xu second-order derivative of running cost w.r.t. state and input
   */
  virtual void calcRunningCostDeriv(double t,
                                    const StateDimVector & x,
                                    const InputDimVector & u,
                                    Eigen::Ref<StateDimVector> running_cost_deriv_x,
                                    Eigen::Ref<InputDimVector> running_cost_deriv_u,
                                    Eigen::Ref<StateStateDimMatrix> running_cost_deriv_xx,
                                    Eigen::Ref<InputInputDimMatrix> running_cost_deriv_uu,
                                    Eigen::Ref<StateInputDimMatrix> running_cost_deriv_xu) const = 0;

  /**
   * @brief Calculate first-order derivatives of terminal cost.
   * @param t time [sec]
   * @param x state
   * @param terminal_cost_deriv_x first-order derivative of terminal cost w.r.t. state
   */
  virtual void calcTerminalCostDeriv(double t,
                                     const StateDimVector & x,
                                     Eigen::Ref<StateDimVector> terminal_cost_deriv_x) const = 0;

  /**
   * @brief Calculate first-order and second-order derivatives of terminal cost.
   * @param t time [sec]
   * @param x state
   * @param terminal_cost_deriv_x first-order derivative of terminal cost w.r.t. state
   * @param terminal_cost_deriv_xx second-order derivative of terminal cost w.r.t. state
   */
  virtual void calcTerminalCostDeriv(double t,
                                     const StateDimVector & x,
                                     Eigen::Ref<StateDimVector> terminal_cost_deriv_x,
                                     Eigen::Ref<StateStateDimMatrix> terminal_cost_deriv_xx) const = 0;

protected:
  // Discretization timestep [sec]
  const double dt_ = 0;
};
}  // namespace ddp
}  // namespace nmpc_controller
}  // namespace controller
}  // namespace control
}  // namespace autonomy
