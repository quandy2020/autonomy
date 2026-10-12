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
 * @file double_integrator.hpp
 * @brief Open-loop double-integrator OCP.
 *
 * Declarations only. The linear dynamics and the SLQ solve live in
 * double_integrator.cpp.
 */

#pragma once

#include <string>

/**
 * @brief Double-integrator example. This namespace is not part of libbolza.
 */
namespace double_integrator {

/**
 * @brief Drive a unit-mass point from the origin to p = 1 and stop.
 *
 * State x = [p, pdot], input u = pddot. Dynamics are the linear pair
 * A = [[0, 1], [0, 0]], B = [0, 1]. The horizon is 3 s.
 */
class DoubleIntegrator {
 public:
  /**
   * @brief Use the double-integrator URDF shipped with the examples.
   *
   * The path is bolza::robotic_assets::GetPath() plus
   * `double_integrator/urdf/double_integrator.urdf`. The file identifies the
   * asset; the OCP uses the analytic linear model.
   */
  DoubleIntegrator();

  /**
   * @brief Record an asset path and solve the same linear model.
   * @param urdf_path Path logged with the result. The dynamics do not parse it.
   */
  explicit DoubleIntegrator(std::string urdf_path);

  /**
   * @brief Solve and log the trajectory.
   * @return 0 when the terminal state is within 0.05 of [1, 0], 1 otherwise.
   */
  int Run();

 private:
  std::string urdf_path_;
};

}  // namespace double_integrator
