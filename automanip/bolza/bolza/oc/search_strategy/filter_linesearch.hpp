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
 * @file filter_linesearch.hpp
 * @brief Checks that the step is accepted.
 */


#include <bolza/core/types.hpp>
#include <bolza/oc/oc_data/performance_index.hpp>

namespace bolza {

/*
 * Filter linesearch based on:
 * "On the implementation of an interior-point filter line-search algorithm for large-scale nonlinear programming"
 * https://link.springer.com/article/10.1007/s10107-004-0559-y
 *
 * step acceptance criteria with c = costs, g = the norm of constraint violation, and w = [x; u]
 */
struct FilterLinesearch {
  enum class StepType { UNKNOWN, CONSTRAINT, DUAL, COST, ZERO };

  scalar_t g_max = 1e6;          // (1): IF g{i+1} > g_max REQUIRE g{i+1} < (1-gamma_c) * g{i}
  scalar_t g_min = 1e-6;         // (2): ELSE IF (g{i} < g_min AND g{i+1} < g_min AND dc/dw'{i} * delta_w < 0) REQUIRE Armijo condition
  scalar_t gamma_c = 1e-6;       // (3): ELSE REQUIRE c{i+1} < (c{i} - gamma_c * g{i}) OR g{i+1} < (1-gamma_c) * g{i}
  scalar_t armijoFactor = 1e-4;  // Armijo condition: c{i+1} < c{i} + armijoFactor * armijoDescentMetric{i}

  /**
   * @brief Checks that the step is accepted.
   * @param [in] baselinePerformance: The zero step PerformanceIndex
   * @param [in] stepPerformance: The step PerformanceIndex
   * @param [in] armijoDescentMetric: The step Armijo descent metric defined as dc/dw' * delta_w
   * @return Result value.
   */
  std::pair<bool, StepType> AcceptStep(const PerformanceIndex& baselinePerformance, const PerformanceIndex& stepPerformance,
                                       scalar_t armijoDescentMetric) const;

  /**
   * @brief Compute total constraint violation.
   * @param [in] performance: Performance.
   * @return Result value.
   */
  static scalar_t TotalConstraintViolation(const PerformanceIndex& performance) {
    /**
     * @brief Sqrt.
     * @param [in] equalityConstraintsSSE: Equality Constraints SSE.
     * @return Result value.
     */
    return std::sqrt(performance.dynamicsViolationSSE + performance.equalityConstraintsSSE);
  }
};

/**
 * @brief Transforms the StepType to string.
 * @param [in] stepType: Step Type.
 * @return Result value.
 */
std::string ToString(const FilterLinesearch::StepType& stepType);

/**
 * Computes the Armijo descent metric that determines if the solution is a descent direction for the cost. It calculates sum of
 * gradient(cost).dot([dx; du]) over the trajectory.
 *
 * @param [in] cost: The quadratic approximation of the cost.
 * @param [in] deltaXSol: The state trajectory of the QP subproblem solution.
 * @param [in] deltaUSol: The input trajectory of the QP subproblem solution.
 * @return The Armijo descent metric.
 */
scalar_t ArmijoDescentMetric(const std::vector<ScalarFunctionQuadraticApproximation>& cost, const vector_array_t& deltaXSol,
                             const vector_array_t& deltaUSol);

}  // namespace bolza
