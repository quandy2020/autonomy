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
 * @file performance_index.hpp
 * @brief Defines the performance indices for a rollout.
 */


#include <ostream>

#include <bolza/core/types.hpp>
#include <bolza/core/model_data/metrics.hpp>

namespace bolza {

/**
 * Defines the performance indices for a rollout
 */
struct PerformanceIndex {
  /** The merit function of a rollout. */
  scalar_t merit = 0.0;

  /** The total cost of a rollout. */
  scalar_t cost = 0.0;

  /** Sum of Squared Error (SSE) of the dual feasibilities:
   * - Final: squared norm of violation in the dual feasibilities
   * - PreJumps: sum of squared norm of violation in the dual feasibilities
   * - Intermediates: sum of squared norm of violation in the dual feasibilities
   */
  scalar_t dualFeasibilitiesSSE = 0.0;

  /** Sum of Squared Error (SSE) of system dynamics violation */
  scalar_t dynamicsViolationSSE = 0.0;

  /** Sum of Squared Error (SSE) of equality constraints:
   * - Final: squared norm of violation in state equality constraints
   * - PreJumps: sum of squared norm of violation in state equality constraints
   * - Intermediates: Integral of squared norm violation in state/state-input equality constraints
   */
  scalar_t equalityConstraintsSSE = 0.0;

  /** Sum of Squared Error (SSE) of inequality constraints:
   * - Final: squared norm of violation in state inequality constraints
   * - PreJumps: sum of squared norm of violation in state inequality constraints
   * - Intermediates: Integral of squared norm violation in state/state-input inequality constraints
   */
  scalar_t inequalityConstraintsSSE = 0.0;

  /** Sum of equality Lagrangians:
   * - Final: penalty for violation in state equality constraints
   * - PreJumps: penalty for violation in state equality constraints
   * - Intermediates: penalty for violation in state/state-input equality constraints
   */
  scalar_t equalityLagrangian = 0.0;

  /** Sum of inequality Lagrangians:
   * - Final: penalty for violation in state inequality constraints
   * - PreJumps: penalty for violation in state inequality constraints
   * - Intermediates: penalty for violation in state/state-input inequality constraints
   */
  scalar_t inequalityLagrangian = 0.0;

  /** Add performance indices. */
  PerformanceIndex& operator+=(const PerformanceIndex& rhs);

  /** Multiply by a scalar. */
  PerformanceIndex& operator*=(const scalar_t c);

  /**
   * @brief Returns true if *this is approximately equal to other, within the precision determined by prec.
   * @param [in] other: Other.
   * @param [in] prec: Prec.
   * @return True on success.
   */
  bool isApprox(const PerformanceIndex& other, const scalar_t prec = 1e-8) const;
};

/** Add performance indices. */
inline PerformanceIndex operator+(PerformanceIndex lhs, const PerformanceIndex& rhs) {
  lhs += rhs;  // copied lhs, add rhs to it.
  return lhs;
}

/** Multiply by a scalar. */
inline PerformanceIndex operator*(PerformanceIndex lhs, const scalar_t c) {
  lhs *= c;  // copied lhs
  return lhs;
}

/** Multiply by a scalar. */
inline PerformanceIndex operator*(const scalar_t c, PerformanceIndex rhs) {
  rhs *= c;  // copied rhs
  return rhs;
}

/**
 * @brief Swaps performance indices.
 * @param [in,out] lhs: Lhs.
 * @param [in,out] rhs: Rhs.
 */
void swap(PerformanceIndex& lhs, PerformanceIndex& rhs);

/**
 * @brief Computes the PerformanceIndex based on a given continuous-time Metrics.
 * @param [in] m: M.
 * @return Result value.
 */
PerformanceIndex ToPerformanceIndex(const Metrics& m);

/**
 * @brief Computes the PerformanceIndex based on a given discrete-time Metrics.
 * @param [in] m: M.
 * @param [in] dt: Dt.
 * @return Result value.
 */
PerformanceIndex ToPerformanceIndex(const Metrics& m, const scalar_t dt);

/** Overloads the stream insertion operator. */
std::ostream& operator<<(std::ostream& stream, const PerformanceIndex& performanceIndex);

}  // namespace bolza
