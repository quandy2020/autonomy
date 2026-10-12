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
 * @file manipulator.hpp
 * @brief Open-loop joint-velocity tracking for a manipulator URDF.
 */

#pragma once

#include <string>

/**
 * @brief Manipulator example. This namespace is not part of libbolza.
 */
namespace manipulator {

/**
 * @brief Move the Franka Panda joints to an alternating posture.
 *
 * State x = q in R^{nv}, input u = qdot, so the dynamics are x_dot = u.
 * The target alternates +0.15 and -0.15 rad across the velocity coordinates.
 * The horizon is 1 s.
 */
class Manipulator {
 public:
  /**
   * @brief Use the Franka Panda URDF shipped with the examples.
   *
   * The path is bolza::robotic_assets::GetPath() plus
   * `mobile_manipulator/franka/urdf/panda.urdf`.
   */
  Manipulator();

  /**
   * @brief Track joints of the URDF at @p urdf_path.
   * @param urdf_path Manipulator URDF. nv must be positive.
   */
  explicit Manipulator(std::string urdf_path);

  /**
   * @brief Solve and log the trajectory.
   * @return 0 when the terminal joint vector is within 0.05 of the target.
   */
  int Run();

 private:
  std::string urdf_path_;
};

}  // namespace manipulator
