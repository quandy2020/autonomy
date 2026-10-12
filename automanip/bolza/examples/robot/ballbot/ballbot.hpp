/*
 * Copyright 2026 Bolza contributors duyongquan (quandy2020@126.com)
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

/**
 * @file ballbot.hpp
 * @brief Open-loop ballbot OCP: translate, then stand upright.
 *
 * Declarations only. Dynamics, the wheel-torque map, and the SLQ solve
 * live in ballbot.cpp.
 */

#pragma once

#include <string>

#include <pinocchio/fwd.hpp>
#include <pinocchio/multibody/data.hpp>
#include <pinocchio/multibody/model.hpp>

#include <bolza/core/types.hpp>

/**
 * @brief Ballbot example. This namespace is not part of libbolza.
 *
 * One type loads the URDF, builds the optimal-control problem, solves it,
 * and prints the trajectory.
 */
namespace ballbot {

/**
 * @brief One open-loop solve for the balancing ballbot.
 *
 * The state is ten-dimensional:
 *   [px, py, yaw, pitch, roll, vx, vy, yaw_rate, pitch_rate, roll_rate]
 * The input is three omnidirectional-wheel torques. Generalized coordinates
 * follow the joint order of the ballbot URDF.
 *
 * Three wheel torques are mapped to generalized forces. Pinocchio ABA then
 * returns the joint accelerations, so the flow map is x_dot = [v, q_ddot].
 * The task is to translate 0.2 m along x in 2 s and return to upright.
 * That flow map is linearized by central differences and solved with bolza SLQ.
 *
 * Copying is disabled because data_ is written on every flow evaluation.
 */
class Ballbot {
 public:
  /**
   * @brief Load the ballbot URDF shipped with the examples.
   *
   * The path is bolza::robotic_assets::GetPath() plus `ballbot/urdf/ballbot.urdf`.
   *
   * @throws std::runtime_error If the URDF cannot be opened or parsed.
   */
  Ballbot();

  /**
   * @brief Load a ballbot URDF from disk.
   *
   * Joint limits written as effort="-1.0" or velocity="-1.0" are replaced in
   * memory with a finite value. The file on disk is left unchanged.
   *
   * @param urdf_path Path to the ballbot URDF.
   * @throws std::runtime_error If the URDF cannot be opened or parsed.
   */
  explicit Ballbot(std::string urdf_path);

  /** @brief Pinocchio work data is mutated during rollout, so copying is disabled. */
  Ballbot(const Ballbot&) = delete;

  /** @brief Pinocchio work data is mutated during rollout, so copying is disabled. */
  Ballbot& operator=(const Ballbot&) = delete;

  /**
   * @brief Solve the translate-and-balance problem and print the trajectory.
   *
   * Failure means the model does not have five velocity coordinates, SLQ did
   * not converge, or the terminal state is farther than 0.15 from the target.
   * On success, a few samples of pose and wheel torque are printed.
   *
   * @return 0 when the target is reached, 1 otherwise.
   */
  int Run();

 private:
  /**
   * @brief Nonlinear state derivative.
   * @param state Ten-dimensional state. The first five entries are positions, the last five are velocities.
   * @param input Three wheel torques.
   * @return [v, q_ddot].
   */
  bolza::vector_t ComputeFlowMap(const bolza::vector_t& state, const bolza::vector_t& input);

  std::string urdf_path_;
  pinocchio::Model model_;
  pinocchio::Data data_;
};

}  // namespace ballbot
