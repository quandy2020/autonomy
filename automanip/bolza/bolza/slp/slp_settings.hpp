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
 * @file slp_settings.hpp
 * @brief Multiple-shooting SLP (Successive Linear Programming) settings.
 */


#include <bolza/core/types.hpp>
#include <bolza/core/integration/sensitivity_integrator.hpp>

#include "bolza/slp/pipg/pipg_settings.hpp"

namespace bolza {
namespace slp {

/** Multiple-shooting SLP (Successive Linear Programming) settings */
struct Settings {
  size_t slpIteration = 10;     // Maximum number of SLP iterations
  size_t scalingIteration = 3;  // Number of pre-conditioning iterations
  scalar_t deltaTol = 1e-6;     // Termination condition : RMS update of x(t) and u(t) are both below this value
  scalar_t costTol = 1e-4;      // Termination condition : (cost{i+1} - (cost{i}) < costTol AND constraints{i+1} < g_min

  // Linesearch - step size rules
  scalar_t alpha_decay = 0.5;  // multiply the step size by this factor every time a linesearch step is rejected.
  scalar_t alpha_min = 1e-4;   // terminate linesearch if the attempted step size is below this threshold

  // Linesearch - step acceptance criteria with c = costs, g = the norm of constraint violation, and w = [x; u]
  scalar_t g_max = 1e6;          // (1): IF g{i+1} > g_max REQUIRE g{i+1} < (1-gamma_c) * g{i}
  scalar_t g_min = 1e-6;         // (2): ELSE IF (g{i} < g_min AND g{i+1} < g_min AND dc/dw'{i} * delta_w < 0) REQUIRE armijo condition
  scalar_t armijoFactor = 1e-4;  // Armijo condition: c{i+1} < c{i} + armijoFactor * dc/dw'{i} * delta_w
  scalar_t gamma_c = 1e-6;       // (3): ELSE REQUIRE c{i+1} < (c{i} - gamma_c * g{i}) OR g{i+1} < (1-gamma_c) * g{i}

  // Discretization method
  scalar_t dt = 0.01;  // user-defined time discretization
  SensitivityIntegratorType integratorType = SensitivityIntegratorType::RK2;

  // Inequality penalty relaxed barrier parameters
  scalar_t inequalityConstraintMu = 0.0;
  scalar_t inequalityConstraintDelta = 1e-6;

  // Extract the Lagrange multiplier of the projected state-input constraint Cx+Du+e
  bool extractProjectionMultiplier = false;

  // Printing
  bool printSolverStatus = false;      // Print HPIPM status after solving the QP subproblem
  bool printSolverStatistics = false;  // Print benchmarking of the multiple shooting method
  bool printLinesearch = false;        // Print linesearch information

  // Threading
  size_t nThreads = 4;
  int threadPriority = 50;

  // LP subproblem solver settings
  pipg::Settings pipgSettings = pipg::Settings();
};

/**
 * Loads the multiple shooting SLP settings from a given file.
 *
 * @param [in] filename: File name which contains the configuration data.
 * @param [in] fieldName: Field name which contains the configuration data.
 * @param [in] verbose: Flag to determine whether to print out the loaded settings or not.
 * @return The settings
 */
Settings LoadSettings(const std::string& filename, const std::string& fieldName = "multiple_shooting", bool verbose = true);

}  // namespace slp
}  // namespace bolza
