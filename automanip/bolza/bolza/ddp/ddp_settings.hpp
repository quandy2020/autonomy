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
 * @file ddp_settings.hpp
 * @brief Enum used in selecting either SLQ, ILQR algorithms.
 */


#include <string>

#include <bolza/core/types.hpp>
#include <bolza/core/integration/integrator.hpp>

#include "bolza/ddp/search_strategy/strategy_settings.hpp"

namespace bolza {
namespace ddp {

/**
 * @brief The DDP algorithm enum
 * Enum used in selecting either SLQ, ILQR algorithms.
 */
enum class Algorithm { SLQ, ILQR };

/**
 * @brief /**.
 *
 * Get string name of DDP algorithm type
 *
 * /
 * @param [in] type: DDP algorithm type enum
 * @return Result value.
 */
std::string ToAlgorithmName(Algorithm type);

/**
 * @brief /**.
 *
 * Get DDP algorithm type from string name, useful for reading config file
 *
 * /
 * @param [in] name: DDP algorithm name
 * @return Result value.
 */
Algorithm FromAlgorithmName(std::string name);

/**
 * This structure contains the settings for the DDP algorithm.
 */
struct Settings {
  /** It should be either SLQ or ILQR */
  Algorithm algorithm_ = Algorithm::SLQ;

  /** Number of threads used in the multi-threading scheme. */
  size_t nThreads_ = 1;
  /** Priority of threads used in the multi-threading scheme. */
  int threadPriority_ = 99;

  /** Maximum number of iterations of DDP. */
  size_t maxNumIterations_ = 15;
  /** This value determines the termination condition based on the minimum relative changes of the cost. */
  scalar_t minRelCost_ = 1e-3;
  /** This value determines the tolerance of constraint's ISE (Integral of Square Error). */
  scalar_t constraintTolerance_ = 1e-3;

  /** This value determines to display the log output DDP. */
  bool displayInfo_ = false;
  /** This value determines to display the a summary log of DDP. */
  bool displayShortSummary_ = false;
  /** Check the numerical stability of the algorithms for debugging purpose. */
  bool checkNumericalStability_ = true;
  /** Printing rollout trajectory for debugging. */
  bool debugPrintRollout_ = false;

  /** This value determines the absolute tolerance error for ode solvers. */
  scalar_t absTolODE_ = 1e-9;
  /** This value determines the relative tolerance error for ode solvers. */
  scalar_t relTolODE_ = 1e-6;
  /** This value determines the maximum number of integration points per a second for ode solvers. */
  size_t maxNumStepsPerSecond_ = 10000;
  /** The integration time step for Riccati equation which is used for fixed timestep integration scheme. */
  scalar_t timeStep_ = 1e-2;
  /** The backward pass integrator type: SLQ uses it for solving Riccati equation and ILQR uses it for discretizing LQ approximation. */
  IntegratorType backwardPassIntegratorType_ = IntegratorType::ODE45;

  /** The initial coefficient of the quadratic penalty function in the merit function. It should be greater than one. */
  scalar_t constraintPenaltyInitialValue_ = 2.0;
  /** The rate that the coefficient of the quadratic penalty function in the merit function grows. It should be greater than one. */
  scalar_t constraintPenaltyIncreaseRate_ = 2.0;

  /** If true, terms of the Riccati equation will be pre-computed before interpolation in the flow-map */
  bool preComputeRiccatiTerms_ = true;

  /** Use either the optimized control policy (true) or the optimized state-input trajectory (false). */
  bool useFeedbackPolicy_ = false;

  /** The risk sensitivity coefficient for risk aware DDP. */
  scalar_t riskSensitiveCoeff_ = 0.0;

  /** Determines the strategy for solving the subproblem. There are two choices line-search strategy and levenberg_marquardt strategy. */
  search_strategy::Type strategy_ = search_strategy::Type::LINE_SEARCH;
  /** The line-search strategy settings. */
  line_search::Settings lineSearch_;
  /** The levenberg_marquardt strategy settings. */
  levenberg_marquardt::Settings levenbergMarquardt_;

};  // end of ddp::Settings

/**
 * This function loads the "ddp::Settings" variables from a config file. This file contains the settings for the SQL and OCS2 algorithms.
 * Here, we use the INFO format which was created specifically for the property tree library (refer to www.goo.gl/fV3yWA).
 *
 * It has the following format: <br>
 * slq  <br>
 * {  <br>
 *   maxIteration        Value    <br>
 *   minLearningRate     Value    <br>
 *   maxLearningRate     Value    <br>
 *   minRelCost          Value    <br>
 *   (and so on for the other fields) <br>
 * }  <br>
 *
 * If a value for a specific field is not defined it will set to the default value defined in "ddp::Settings".
 *
 * @param [in] filename: File name which contains the configuration data.
 * @param [in] fieldName: Field name which contains the configuration data.
 * @param [in] verbose: Flag to determine whether to print out the loaded settings or not (The default is true).
 * @return The DDP settings.
 */
Settings LoadSettings(const std::string& filename, const std::string& fieldName = "ddp", bool verbose = true);

}  // namespace ddp
}  // namespace bolza
