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
 * @file pipg_settings.hpp
 * @brief Maximum number of iterations of PIPG.
 */


#include <bolza/core/types.hpp>

namespace bolza {
namespace pipg {

struct Settings {
  /** Maximum number of iterations of PIPG. */
  size_t maxNumIterations = 3000;
  /** Termination criteria. **/
  scalar_t absoluteTolerance = 1e-3;
  scalar_t relativeTolerance = 1e-2;
  /** Number of iterations between consecutive calculation of termination conditions. **/
  size_t checkTerminationInterval = 1;
  /** The static lower bound of the cost hessian H. **/
  scalar_t lowerBoundH = 5e-6;
  /** This value determines to display the a summary log. */
  bool displayShortSummary = false;
};

/**
 * Loads the PIPG settings from a given file.
 *
 * @param [in] filename: File name which contains the configuration data.
 * @param [in] fieldName: Field name which contains the configuration data.
 * @param [in] verbose: Flag to determine whether to print out the loaded settings or not.
 * @return The settings
 */
Settings LoadSettings(const std::string& filename, const std::string& fieldName = "pipg", bool verbose = true);

}  // namespace pipg
}  // namespace bolza
