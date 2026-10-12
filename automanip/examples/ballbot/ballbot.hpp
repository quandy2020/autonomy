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
 * @file ballbot.hpp
 * @brief Ballbot example: CppAD dynamics and quadratic cost via bolza::Problem.
 *
 * State @f$ x \in \mathbb{R}^{10} @f$ (ball XY, ZYX Euler angles, rates),
 * input @f$ u \in \mathbb{R}^{3} @f$ (wheel torques).
 *
 * Autolink entry: @c ballbot.cpp. Dynamics: @c dynamics/ballbot_system_dynamics.hpp.
 * Optional solver: pass @c slp as the second CLI argument to use @c SlpMpc.
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
#include <bolza/slp/slp_settings.hpp>
#include <bolza/tools/common/robot_interface.hpp>

namespace bolza {
namespace ballbot {

/** State dimension. */
inline constexpr std::size_t STATE_DIM = 10;

/** Input dimension (three wheel torques). */
inline constexpr std::size_t INPUT_DIM = 3;

/** Number of actuated / visualized joints in the URDF. */
inline constexpr std::size_t JOINTS_DOF_NUM = 5;

/**
 * @brief Ballbot MPC example: load task.info and build OCP / rollout / solver settings.
 *
 * Supports Gauss-Newton DDP and SLP settings loaded from the same INFO file.
 * Startup target defaults to the loaded initial state.
 *
 * @see bolza::Problem, bolza::GaussNewtonDDPMPC, bolza::SlpMpc, bolza::examples::RunDemo
 */
class Ballbot final : public RobotInterface {
 public:
  /**
   * @brief Construct the problem and supporting objects from mpc/task.info.
   * @param [in] taskFile: Absolute path to the INFO configuration file.
   * @param [in] libraryFolder: Directory for CppAD-generated libraries.
   * @param [in] verbose: If true, print load path, matrices, and section settings.
   * @throw std::invalid_argument If @p taskFile does not exist.
   */
  explicit Ballbot(const std::string& taskFile,
                   const std::string& libraryFolder = "/tmp/automanip_ballbot",
                   bool verbose = true);

  /**
   * @brief Mutable Problem facade.
   * @return Reference to the owned @c Problem.
   */
  Problem& GetProblem() { return problem_; }

  /**
   * @brief Const Problem facade.
   * @return Const reference to the owned @c Problem.
   */
  const Problem& GetProblem() const { return problem_; }

  /**
   * @brief Startup target (hover / balance at the loaded initial state).
   * @return Const reference to @c Problem::GetInitialState().
   */
  const vector_t& GetInitialTarget() const { return problem_.GetInitialState(); }

  /**
   * @brief SLP solver settings.
   * @return Mutable reference to @c slp::Settings.
   */
  slp::Settings& SlpSettings() { return slpSettings_; }

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
   * @brief Input initializer for DDP / SLP warm-start.
   * @return Const reference to @c DefaultInitializer(INPUT_DIM).
   */
  const Initializer& GetInitializer() const override { return *initializer_; }

 private:
  ddp::Settings ddpSettings_;  ///< Loaded from the @c ddp section of task.info.
  mpc::Settings mpcSettings_;  ///< Loaded from the @c mpc section of task.info.
  slp::Settings slpSettings_;  ///< Loaded from the @c slp section of task.info.

  Problem problem_;

  std::unique_ptr<RolloutBase> rollout_;
  std::unique_ptr<Initializer> initializer_;
};

}  // namespace ballbot
}  // namespace bolza
