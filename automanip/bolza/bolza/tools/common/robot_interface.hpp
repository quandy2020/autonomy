/*
 * Copyright 2026 Autodriver contributors duyongquan (quandy2020@126.com)
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
 * @file robot_interface.hpp
 * @brief This class implements an interface class to all the robotic examples.
 */


#include <memory>

#include <bolza/core/types.hpp>
#include <bolza/core/dynamics/system_dynamics_base.hpp>
#include <bolza/core/initialization/initializer.hpp>
#include <bolza/oc/oc_problem/optimal_control_problem.hpp>
#include <bolza/oc/synchronized_module/reference_manager_interface.hpp>

namespace bolza {

/**
 * This class implements an interface class to all the robotic examples.
 *
 * The lifetime of the returned objects is tied to the lifetime of the robot interface.
 * The exposed objects are not thread-safe and should be cloned to get an exclusive copy.
 */
class RobotInterface {
 public:
  /** Constructor */
  RobotInterface() = default;

  /** Destructor */
  virtual ~RobotInterface() = default;

  /**
   * Gets the ReferenceManager.
   * @return a shared pointer to the ReferenceManager.
   */
  virtual std::shared_ptr<ReferenceManagerInterface> GetReferenceManagerPtr() const { return nullptr; }

  /**
   * @brief Get the optimal control problem definition
   * @return reference to the problem object
   */
  virtual const OptimalControlProblem& GetOptimalControlProblem() const = 0;

  /**
   * @brief getInitializer
   * @return reference to the internal solver initializer
   */
  virtual const Initializer& GetInitializer() const = 0;
};

}  // namespace bolza
