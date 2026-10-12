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
 * @file slp_solver_status.hpp
 * @brief Different types of convergence.
 */


#include <string>

#include <bolza/core/types.hpp>
#include <bolza/oc/oc_data/performance_index.hpp>
#include <bolza/oc/search_strategy/filter_linesearch.hpp>

namespace bolza {
namespace slp {

/** Different types of convergence */
enum class Convergence { FALSE, ITERATIONS, STEPSIZE, METRICS, PRIMAL };

/** Struct to contain the result and logging data of the stepsize computation */
struct StepInfo {
  // Step size and type
  scalar_t stepSize = 0.0;
  FilterLinesearch::StepType stepType = FilterLinesearch::StepType::UNKNOWN;

  // Step in primal variables
  scalar_t dx_norm = 0.0;  // norm of the state trajectory update
  scalar_t du_norm = 0.0;  // norm of the input trajectory update

  // Performance result after the step
  PerformanceIndex performanceAfterStep;
  scalar_t totalConstraintViolationAfterStep;  // constraint metric used in the line search
};

/**
 * @brief Transforms pipg::Convergence to string.
 * @param [in] convergence: Convergence.
 * @return Result value.
 */
inline std::string ToString(const Convergence& convergence) {
  switch (convergence) {
    case Convergence::ITERATIONS:
      return "Maximum number of iterations reached";
    case Convergence::STEPSIZE:
      return "Step size below minimum";
    case Convergence::METRICS:
      return "Cost decrease and constraint satisfaction below tolerance";
    case Convergence::PRIMAL:
      return "Primal update below tolerance";
    case Convergence::FALSE:
    default:
      return "Not Converged";
  }
}

}  // namespace slp
}  // namespace bolza
