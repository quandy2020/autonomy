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
 * @file rollout_settings.hpp
 * @brief This structure contains the settings for forward rollout algorithms.
 */


#include <iostream>
#include <string>

#include <bolza/core/types.hpp>
#include <bolza/core/integration/integrator.hpp>
#include "bolza/oc/rollout/root_finder_type.hpp"

namespace bolza {
namespace rollout {

/**
 * This structure contains the settings for forward rollout algorithms.
 */
struct Settings {
  /** This value determines the absolute tolerance error for ode solvers. */
  scalar_t absTolODE = 1e-9;
  /** This value determines the relative tolerance error for ode solvers. */
  scalar_t relTolODE = 1e-6;
  /** This value determines the maximum number of integration points per a second for ode solvers. */
  size_t maxNumStepsPerSecond = 10000;
  /** The integration time step used in the fixed time-step rollout methods */
  scalar_t timeStep = 1e-2;
  /** Rollout integration scheme type */
  IntegratorType integratorType = IntegratorType::ODE45;

  /** Whether to check that the rollout is numerically stable */
  bool checkNumericalStability = false;
  /** Whether to run controller again after integration to construct input trajectory */
  bool reconstructInputTrajectory = true;

  /** Which of the RootFinding algorithms to use in StateRollout
   * 		0:		Anderson & Björck		(default)
   * 		1:		Pegasus
   * 		2:		Illinois
   * 		3:		Regula Falsi
   */
  RootFinderType rootFindingAlgorithm = RootFinderType::ANDERSON_BJORCK;
  /** This value determines the maximum number of iterations, per event, allowed in state triggered rollout to find
   *  the guard surface zero crossing.  */
  int maxSingleEventIterations = 10;
  /** Whether to use the trajectory spreading controller in state triggered rollout */
  bool useTrajectorySpreadingController = false;
};

/**
 * @brief This function loads the "rollout::Settings" variables from a config file. This file contains the settings for the Rollout algorithms.
 *
 * Here, we use the INFO format which was created specifically for the property tree library (refer to www.goo.gl/fV3yWA).
 *
 * It has the following format: <br>
 *
 * rollout  <br>
 *
 * {  <br>
 *
 * absTolODE                Value   <br>
 *
 * relTolODE                Value   <br>
 *
 * maxNumStepsPerSecond     Value   <br>
 *
 * timeStep                 Value   <br>
 *
 * (and so on for the other fields) <br>
 *
 * }  <br>
 *
 * If a value for a specific field is not defined it will set to the default value defined in "rollout::Settings".
 * @param [in] filename: File name which contains the configuration data.
 * @param [in] fieldName: Field name which contains the configuration data.
 * @param [in] verbose: Flag to determine whether to print out the loaded settings or not (The default is true).
 * @return Result value.
 */
Settings LoadSettings(const std::string& filename, const std::string& fieldName = "rollout", bool verbose = true);

}  // namespace rollout
}  // namespace bolza
