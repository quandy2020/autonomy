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
 * @file helpers.hpp
 * @brief Compute 2-norm of the trajectory: sqrt(sum_i v[i]^2).
 */


#include <bolza/core/types.hpp>

#include "bolza/oc/oc_data/performance_index.hpp"
#include "bolza/oc/oc_data/primal_solution.hpp"
#include "bolza/oc/oc_data/problem_metrics.hpp"
#include "bolza/oc/oc_data/time_discretization.hpp"

namespace bolza {
namespace multiple_shooting {

/**
 * @brief Compute 2-norm of the trajectory: sqrt(sum_i v[i]^2).
 * @param [in] v: V.
 * @return Result value.
 */
inline scalar_t TrajectoryNorm(const vector_array_t& v) {
  scalar_t norm = 0.0;
  for (const auto& vi : v) {
    norm += vi.squaredNorm();
  }
  /**
   * @brief Sqrt.
   * @param [in] norm: Norm.
   * @return Result value.
   */
  return std::sqrt(norm);
}

/** Increment the given trajectory as: vNew[i] = v[i] + alpha * dv[i]. It assumes that vNew is already resized to size of v. */
template <typename Type>
/**
 * @brief Increment Trajectory.
 * @param [in] v: V.
 * @param [in] dv: Dv.
 * @param [in] alpha: Alpha.
 * @param [in,out] vNew: V New.
 */
void IncrementTrajectory(const std::vector<Type>& v, const std::vector<Type>& dv, const scalar_t alpha, std::vector<Type>& vNew) {
  assert(v.size() == dv.size());
  if (v.size() != vNew.size()) {
    /**
     * @brief Runtime error.
     * @return Result value.
     */
    throw std::runtime_error("[incrementTrajectory] Resize vNew to the size of v!");
  }

  for (int i = 0; i < v.size(); i++) {
    if (dv[i].size() > 0) {  // account for absence of inputs at events.
      vNew[i] = v[i] + alpha * dv[i];
    } else {
      vNew[i] = Type();
    }
  }
}

/**
 * Re-map the projected input back to the original space.
 *
 * @param [in] constraintsProjection: The constraints projection.
 * @param [in] deltaXSol: The state trajectory of the QP subproblem solution.
 * @param [in,out] deltaUSol: The input trajectory of the QP subproblem solution.
 */
void RemapProjectedInput(const std::vector<VectorFunctionLinearApproximation>& constraintsProjection, const vector_array_t& deltaXSol,
                         vector_array_t& deltaUSol);

/**
 * @brief Remap Projected Gain.
 * @param [in] constraintsProjection: Constraints Projection.
 * @param [in,out] KMatrices: KMatrices.
 */
void RemapProjectedGain(const std::vector<VectorFunctionLinearApproximation>& constraintsProjection, matrix_array_t& KMatrices);

/**
 * Constructs a primal solution (with a feedforward controller) based the LQ subproblem solution.
 *
 * @param [in] time: The annotated time trajectory
 * @param [in] modeSchedule: The mode schedule.
 * @param [in] x: The state trajectory of the QP subproblem solution.
 * @param [in] u: The input trajectory of the QP subproblem solution.
 * @return The primal solution.
 */
PrimalSolution ToPrimalSolution(const std::vector<AnnotatedTime>& time, ModeSchedule&& modeSchedule, vector_array_t&& x,
                                vector_array_t&& u);

/**
 * Constructs a primal solution (with a linear controller) based the LQ subproblem solution.
 *
 * @param [in] time: The annotated time trajectory
 * @param [in] modeSchedule: The mode schedule.
 * @param [in] x: The state trajectory of the QP subproblem solution.
 * @param [in] u: The input trajectory of the QP subproblem solution.
 * @param [in] KMatrices: The LQR gain trajectory of the QP subproblem solution.
 * @return The primal solution.
 */
PrimalSolution ToPrimalSolution(const std::vector<AnnotatedTime>& time, ModeSchedule&& modeSchedule, vector_array_t&& x, vector_array_t&& u,
                                matrix_array_t&& KMatrices);

/**
 * Constructs a ProblemMetrics from an array of metrics.
 *
 * @param [in] time: The annotated time trajectory
 * @param [in] metrics: The metrics array.
 * @return The ProblemMetrics.
 */
ProblemMetrics ToProblemMetrics(const std::vector<AnnotatedTime>& time, std::vector<Metrics>&& metrics);

}  // namespace multiple_shooting
}  // namespace bolza
