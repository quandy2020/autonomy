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
 * @file double_integrator.hpp
 * @brief 1D double-integrator example: LTI dynamics and quadratic cost via bolza::Problem.
 *
 * State and input:
 * - State @f$ x = [p;\ \dot{p}] \in \mathbb{R}^{2} @f$
 * - Input @f$ u = \ddot{p} \in \mathbb{R} @f$
 * - Continuous dynamics @f$ \dot{x} = A x + B u @f$ with
 *   @f$ A = \begin{bmatrix} 0 & 1 \\ 0 & 0 \end{bmatrix} @f$,
 *   @f$ B = \begin{bmatrix} 0 \\ 1 \end{bmatrix} @f$
 *
 * Autolink visualization entry point: @c double_integrator.cpp in this directory.
 * Headless @c bolza::Solve tutorial:
 * @c examples/tutorial/solve_double_integrator.cpp.
 */

#include <cstddef>
#include <memory>
#include <string>

#include <bolza/core/initialization/initializer.hpp>
#include <bolza/core/types.hpp>
#include <bolza/ddp/ddp_settings.hpp>
#include <bolza/mpc/mpc_settings.hpp>
#include <bolza/oc/rollout/time_triggered_rollout.hpp>
#include <bolza/problem.hpp>
#include <bolza/tools/common/robot_interface.hpp>

namespace bolza {
namespace double_integrator {

/** State dimension: @f$ x = [p;\ \dot{p}] @f$. */
inline constexpr std::size_t STATE_DIM = 2;

/** Input dimension: @f$ u = \ddot{p} @f$. */
inline constexpr std::size_t INPUT_DIM = 1;

/**
 * @brief Double-integrator MPC example interface: load task.info and build OCP / rollout / solver settings.
 *
 * Reads @c initialState, @c finalGoal, weights @c Q / @c R / @c Q_final, and the
 * @c ddp, @c mpc, @c rollout sections from the INFO file; then configures linear
 * dynamics and quadratic running/terminal costs through @ref bolza::Problem.
 * Returned objects live as long as this instance and are consumed by
 * @c GaussNewtonDDPMPC and @c examples::RunDemo.
 *
 * @note The LTI plant needs no CppAD codegen, so no library directory is required.
 * @see bolza::Problem, bolza::GaussNewtonDDPMPC, bolza::examples::RunDemo
 */
class DoubleIntegrator final : public RobotInterface {
 public:
  /**
   * @brief Construct the problem and supporting objects from mpc/task.info.
   * @param [in] taskFile: Absolute path to the INFO configuration file.
   * @param [in] verbose: If true, print load path, weight matrices, and section settings.
   * @throw std::invalid_argument If @p taskFile does not exist.
   */
  explicit DoubleIntegrator(const std::string& taskFile, bool verbose = true);

  /**
   * @brief Mutable Problem facade (dynamics / cost / target / initial state).
   * @return Reference to the owned @c Problem.
   */
  Problem& GetProblem() { return problem_; }

  /**
   * @brief Const Problem facade.
   * @return Const reference to the owned @c Problem.
   */
  const Problem& GetProblem() const { return problem_; }

  /**
   * @brief Constant setpoint used at startup (INFO key @c finalGoal).
   * @return State-sized vector, typically @f$ [p^\star;\ 0] @f$.
   * @note The ReferenceManager buffer may not activate this target until the first
   *       @c PreSolverRun; Autolink initial trajectories should use this accessor
   *       rather than reading the reference directly.
   */
  const vector_t& GetInitialTarget() const { return initialTarget_; }

  /**
   * @brief DDP solver settings (mutable before passing into MPC).
   * @return Mutable reference to @c ddp::Settings.
   */
  ddp::Settings& DdpSettings() { return ddpSettings_; }

  /**
   * @brief MPC loop settings (horizon, frequency, …).
   * @return Mutable reference to @c mpc::Settings.
   */
  mpc::Settings& MpcSettings() { return mpcSettings_; }

  /**
   * @brief Underlying optimal-control problem definition.
   * @return Const reference to the OCP owned by @c Problem.
   */
  const OptimalControlProblem& GetOptimalControlProblem() const override {
    return problem_.GetOptimalControlProblem();
  }

  /**
   * @brief Reference trajectory / mode-schedule manager.
   * @return Shared pointer to the @c ReferenceManager owned with @c Problem.
   */
  std::shared_ptr<ReferenceManagerInterface> GetReferenceManagerPtr() const override {
    return problem_.GetReferenceManager();
  }

  /**
   * @brief Time-triggered rollout (simulation and MRT forward integration).
   * @return Const reference to the owned rollout.
   */
  const RolloutBase& GetRollout() const { return *rollout_; }

  /**
   * @brief Input initializer used for DDP warm-start.
   * @return Const reference to @c DefaultInitializer(INPUT_DIM).
   */
  const Initializer& GetInitializer() const override { return *initializer_; }

 private:
  ddp::Settings ddpSettings_;  ///< Loaded from the @c ddp section of task.info.
  mpc::Settings mpcSettings_;  ///< Loaded from the @c mpc section of task.info.

  Problem problem_;                     ///< Dynamics, costs, target, and initial state.
  vector_t initialTarget_{STATE_DIM};  ///< INFO @c finalGoal for RunDemo startup.

  std::unique_ptr<RolloutBase> rollout_;      ///< @c TimeTriggeredRollout.
  std::unique_ptr<Initializer> initializer_;  ///< @c DefaultInitializer.
};

}  // namespace double_integrator
}  // namespace bolza
