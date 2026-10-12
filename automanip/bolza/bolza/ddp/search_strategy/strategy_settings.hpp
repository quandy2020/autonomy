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
 * @file strategy_settings.hpp
 * @brief Enum used in selecting either LINE_SEARCH, LEVENBERG_MARQUARDT, or TRUST_REGION strategies.
 */


#include <string>

#include <bolza/core/numeric_traits.hpp>
#include <bolza/core/types.hpp>
#include <bolza/ddp/hessian_correction.hpp>

namespace bolza {

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
namespace search_strategy {

/**
 * @brief The DDP strategy enum
 * Enum used in selecting either LINE_SEARCH, LEVENBERG_MARQUARDT, or TRUST_REGION strategies.
 */
enum class Type { LINE_SEARCH, LEVENBERG_MARQUARDT };

/**
 * @brief /**.
 *
 * Get string name of DDP_Strategy type
 *
 * /
 * @param [in] strategy: DDP_Strategy type enum
 * @return Result value.
 */
std::string ToString(Type strategy);

/**
 * @brief /**.
 *
 * Get DDP_Strategy type from string name, useful for reading config file
 *
 * /
 * @param [in] name: DDP_Strategy name
 * @return Result value.
 */
Type FromString(std::string name);

/**
 * This structure contains the settings for the Line-Search strategy.
 */
struct Settings {
  /** This value determines to display the log output DDP. */
  bool displayInfo = false;
  /** Printing rollout trajectory for debugging. */
  bool debugPrintRollout = false;
  /** This value determines the termination condition based on the minimum relative changes of the cost. */
  scalar_t minRelCost = 1e-3;
  /** This value determines the tolerance of constraint's ISE (Integral of Square Error). */
  scalar_t constraintTolerance = 1e-3;
};  // end of Settings

}  // namespace search_strategy

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
namespace line_search {

/**
 * This structure contains the settings for the Line-Search strategy.
 */
struct Settings {
  /** Minimum step length of line-search strategy. */
  scalar_t minStepLength = 0.05;
  /** Maximum step length of line-search strategy. */
  scalar_t maxStepLength = 1.0;
  /** Line-search strategy contraction rate. */
  scalar_t contractionRate = 0.5;
  /** Armijo coefficient, c defined as f(u + a*p) < f(u) + c*a dfdu.dot(p)  */
  scalar_t armijoCoefficient = 1e-4;
  /** The Hessian correction strategy. */
  hessian_correction::Strategy hessianCorrectionStrategy = hessian_correction::Strategy::DIAGONAL_SHIFT;
  /** The multiple used for correcting the Hessian for numerical stability of the Riccati backward pass.*/
  scalar_t hessianCorrectionMultiple = numeric_traits::LimitEpsilon<scalar_t>();
};  // end of Settings

/**
 * @brief /**.
 *
 * This function loads the "Line_Search" variables from a config file.
 *
 * Here, we use the INFO format which was created specifically for the property tree library (refer to www.goo.gl/fV3yWA).
 *
 * /
 * @param [in] filename: File name which contains the configuration data.
 * @param [in] fieldName: Field name which contains the configuration data.
 * @param [in] verbose: Flag to determine whether to print out the loaded settings or not (The default is true).
 * @return Result value.
 */
Settings Load(const std::string& filename, const std::string& fieldName, bool verbose = true);

}  // namespace line_search

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
namespace levenberg_marquardt {

/**
 * This structure contains the settings for the Levenberg-Marquardt strategy.
 */
struct Settings {
  /** Minimum pho (the ratio between actual reduction and predicted reduction) to accept the iteration's solution.
   * minAcceptedPho_ should be [0, 0.25);
   * */
  scalar_t minAcceptedPho = 0.25;
  /** The default ratio of geometric progression for Riccati multiple. */
  scalar_t riccatiMultipleDefaultRatio = 2.0;
  /** The default scalar-factor of geometric progression for Riccati multiple. */
  scalar_t riccatiMultipleDefaultFactor = 1e-6;
  /** Maximum number of successive rejections of the iteration's solution. */
  size_t maxNumSuccessiveRejections = 5;
};  // end of Settings

/**
 * @brief /**.
 *
 * This function loads the "Levenberg_Marquardt" variables from a config file.
 *
 * Here, we use the INFO format which was created specifically for the property tree library (refer to www.goo.gl/fV3yWA).
 *
 * /
 * @param [in] filename: File name which contains the configuration data.
 * @param [in] fieldName: Field name which contains the configuration data.
 * @param [in] verbose: Flag to determine whether to print out the loaded settings or not (The default is true).
 * @return Result value.
 */
Settings Load(const std::string& filename, const std::string& fieldName, bool verbose = true);

}  // namespace levenberg_marquardt

}  // namespace bolza
