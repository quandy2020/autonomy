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
 * @file quadrotor.hpp
 * @brief Open-loop quadrotor OCP in hover deviation coordinates.
 */

#pragma once

#include <string>

#include <bolza/core/types.hpp>

/**
 * @brief Quadrotor example. This namespace is not part of libbolza.
 */
namespace quadrotor {

/**
 * @brief Translate a hovering quadrotor and hold the new position.
 *
 * The rigid-body map is linearized once at hover. The OCP state is the
 * deviation from that hover point:
 *   [x, y, z, roll, pitch, yaw, vx, vy, vz, wx, wy, wz]
 * Input is the deviation of [Fz, Mx, My, Mz] from the hover thrust
 * [m g, 0, 0, 0]. Mass is 0.5 kg. The target deviation is +0.3 m in x and
 * +0.2 m in z. The horizon is 2 s.
 */
class Quadrotor {
 public:
  /**
   * @brief Use the quadrotor URDF shipped with the examples.
   *
   * The path is bolza::robotic_assets::GetPath() plus
   * `quadrotor/urdf/quadrotor.urdf`. Inertial numbers used by the flow map
   * match that file. The URDF itself has no joints.
   */
  Quadrotor();

  /**
   * @brief Record an asset path and solve the same hover problem.
   * @param urdf_path Path logged with the result.
   */
  explicit Quadrotor(std::string urdf_path);

  /**
   * @brief Solve and log the deviation trajectory.
   * @return 0 when hover is an equilibrium and the terminal deviation is within 0.05.
   */
  int Run();

 private:
  /**
   * @brief Nonlinear rigid-body derivative in absolute coordinates.
   * @param state Twelve-dimensional pose and velocity.
   * @param input Thrust and body moments [Fz, Mx, My, Mz].
   * @return State derivative.
   */
  bolza::vector_t ComputeFlowMap(const bolza::vector_t& state, const bolza::vector_t& input) const;

  std::string urdf_path_;
};

}  // namespace quadrotor
