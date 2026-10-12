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
 * @file cartpole.hpp
 * @brief Cart-pole example: nonlinear dynamics and quadratic cost via bolza::Problem.
 *
 * State and input:
 * - State @f$ x = [\theta;\ p;\ \dot{\theta};\ \dot{p}] \in \mathbb{R}^{4} @f$
 * - Input @f$ u \in \mathbb{R} @f$ (horizontal force on the cart)
 *
 * Autolink entry: @c cartpole.cpp. Dynamics: @c dynamics/cart_pole_system_dynamics.hpp.
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
namespace cartpole {

/** State dimension: @f$ x = [\theta;\ p;\ \dot{\theta};\ \dot{p}] @f$. */
inline constexpr std::size_t STATE_DIM = 4;

/** Input dimension: cart force. */
inline constexpr std::size_t INPUT_DIM = 1;

/**
 * @brief Cart-pole MPC example: load task.info and build OCP / rollout / solver settings.
 *
 * Reads @c initialState, @c x_final, weights @c Q / @c R / @c Q_final, plant parameters,
 * input-limit soft constraints, and @c ddp / @c mpc / @c rollout sections. Assembles the
 * OCP through @ref bolza::Problem with CppAD cart-pole dynamics.
 *
 * @see bolza::Problem, bolza::GaussNewtonDDPMPC, bolza::examples::RunDemo
 */
class CartPole final : public RobotInterface {
 public:
  /**
   * @brief Construct the problem and supporting objects from mpc/task.info.
   * @param [in] taskFile: Absolute path to the INFO configuration file.
   * @param [in] libraryFolder: Directory for CppAD-generated libraries.
   * @param [in] verbose: If true, print load path, matrices, and section settings.
   * @throw std::invalid_argument If @p taskFile does not exist.
   */
  explicit CartPole(const std::string& taskFile,
                    const std::string& libraryFolder = "/tmp/automanip_cartpole",
                    bool verbose = true);

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
   * @brief Constant setpoint used at startup (INFO key @c x_final).
   * @return State-sized target vector.
   */
  const vector_t& GetInitialTarget() const { return initialTarget_; }

  /**
   * @brief DDP solver settings.
   * @return Mutable reference to @c ddp::Settings.
   */
  ddp::Settings& DdpSettings() { return ddpSettings_; }

  /**
   * @brief MPC loop settings.
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
   * @brief Time-triggered rollout.
   * @return Const reference to the owned rollout.
   */
  const RolloutBase& GetRollout() const { return *rollout_; }

  /**
   * @brief Input initializer for DDP warm-start.
   * @return Const reference to @c DefaultInitializer(INPUT_DIM).
   */
  const Initializer& GetInitializer() const override { return *initializer_; }

 private:
  ddp::Settings ddpSettings_;  ///< Loaded from the @c ddp section of task.info.
  mpc::Settings mpcSettings_;  ///< Loaded from the @c mpc section of task.info.

  Problem problem_;                     ///< Dynamics, costs, target, and initial state.
  vector_t initialTarget_{STATE_DIM};  ///< INFO @c x_final for RunDemo startup.

  std::unique_ptr<RolloutBase> rollout_;      ///< @c TimeTriggeredRollout.
  std::unique_ptr<Initializer> initializer_;  ///< @c DefaultInitializer.
};

}  // namespace cartpole
}  // namespace bolza
