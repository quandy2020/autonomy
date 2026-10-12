/*
 * Copyright 2026 Automanip contributors duyongquan (quandy2020@126.com)
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

#pragma once

/**
 * @file problem.hpp
 * @brief Problem facade over OptimalControlProblem (dynamics / cost / target / x0).
 */

#include <memory>
#include <string>

#include <bolza/core/cost/state_cost.hpp>
#include <bolza/core/cost/state_input_cost.hpp>
#include <bolza/core/dynamics/system_dynamics_base.hpp>
#include <bolza/core/reference/target_trajectories.hpp>
#include <bolza/core/types.hpp>
#include <bolza/oc/oc_problem/optimal_control_problem.hpp>
#include <bolza/oc/synchronized_module/reference_manager.hpp>

namespace bolza {

/**
 * @brief Lightweight OCP builder for libbolza.
 *
 * Typical use:
 * @code
 * bolza::Problem problem;
 * problem.SetLinearDynamics(A, B);
 * problem.AddQuadraticCost(Q, R);
 * problem.AddQuadraticFinalCost(Qf);
 * problem.SetTarget(x_star, u_star);
 * problem.SetInitialState(x0);
 * @endcode
 */
class Problem {
 public:
  Problem();
  ~Problem();

  Problem(const Problem&) = delete;
  Problem& operator=(const Problem&) = delete;
  Problem(Problem&&) noexcept;
  Problem& operator=(Problem&&) noexcept;

  /**
   * @brief Set system dynamics.
   * @param [in] dynamics Ownership transferred.
   */
  void SetDynamics(std::unique_ptr<SystemDynamicsBase> dynamics);

  /**
   * @brief LTI dynamics ẋ = A x + B u.
   * @param [in] A State matrix.
   * @param [in] B Input matrix.
   */
  void SetLinearDynamics(const matrix_t& A, const matrix_t& B);

  /**
   * @brief Add intermediate state-input cost.
   * @param [in] name Cost name in the collection.
   * @param [in] cost Ownership transferred.
   */
  void AddCost(std::string name, std::unique_ptr<StateInputCost> cost);

  /**
   * @brief Add terminal state cost.
   * @param [in] name Cost name in the collection.
   * @param [in] cost Ownership transferred.
   */
  void AddFinalCost(std::string name, std::unique_ptr<StateCost> cost);

  /**
   * @brief Quadratic intermediate cost with weight matrices Q, R.
   * @param [in] Q State weight.
   * @param [in] R Input weight.
   */
  void AddQuadraticCost(const matrix_t& Q, const matrix_t& R);

  /**
   * @brief Quadratic terminal cost with weight Qf.
   * @param [in] Qf Terminal state weight.
   */
  void AddQuadraticFinalCost(const matrix_t& Qf);

  /**
   * @brief Constant setpoint (x★, u★). Empty u_star → Zero(input_dim).
   * @param [in] x_star Desired state.
   * @param [in] u_star Desired input (optional).
   * @param [in] time Sample time for the setpoint.
   */
  void SetTarget(const vector_t& x_star, const vector_t& u_star = vector_t(), scalar_t time = 0.0);

  /**
   * @brief Set full target trajectories.
   * @param [in] trajectories Desired time / state / input trajectories.
   */
  void SetTarget(TargetTrajectories trajectories);

  /**
   * @brief Set initial state x0 for Solve().
   * @param [in] x0 Initial state.
   */
  void SetInitialState(const vector_t& x0);

  /** @brief Inferred input dimension. */
  size_t GetInputDim() const;

  /** @brief Initial state set via SetInitialState(). */
  const vector_t& GetInitialState() const;

  /** @brief Mutable underlying OCP. */
  OptimalControlProblem& GetOptimalControlProblem();

  /** @brief Const underlying OCP. */
  const OptimalControlProblem& GetOptimalControlProblem() const;

  /** @brief Shared reference manager (mode / targets). */
  std::shared_ptr<ReferenceManager> GetReferenceManager() const;

 private:
  OptimalControlProblem ocp_;
  std::shared_ptr<ReferenceManager> reference_;
  vector_t x0_;
  size_t input_dim_ = 0;
};

}  // namespace bolza
