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
 * @file metrics.hpp
 * @brief The structure contains a term's constraint vector and its associated penalty.
 */


#include "bolza/core/types.hpp"
#include "bolza/core/misc/linear_interpolation.hpp"

namespace bolza {

/** The structure contains a term's constraint vector and its associated penalty */
struct LagrangianMetrics {
  /**
   * @brief Constructs a LagrangianMetrics.
   */
  LagrangianMetrics() : LagrangianMetrics(0.0, vector_t()) {}
  /**
   * @brief Constructs a LagrangianMetrics.
   * @param [in] penaltyArg: Penalty Arg.
   */
  LagrangianMetrics(scalar_t penaltyArg, vector_t constraintArg) : penalty(penaltyArg), Constraint(std::move(constraintArg)) {}

  scalar_t penalty;
  vector_t constraint;
};

/** A const reference view to LagrangianMetrics. This is useful for having an array of references to LagrangianMetrics. */
struct LagrangianMetricsConstRef {
  /**
   * @brief Constructs a LagrangianMetricsConstRef.
   * @param [in] constraint: Constraint.
   */
  LagrangianMetricsConstRef(const LagrangianMetrics& metricsArg) : penalty(metricsArg.penalty), Constraint(metricsArg.constraint) {}
  operator LagrangianMetrics() const { return {penalty, constraint}; }

  const scalar_t& penalty;
  const vector_t& constraint;
};

/**
 * The collection of cost, dynamics violation, constraints, and LagrangianMetrics structure for all possible constraint
 * terms (handled by Lagrangian method) in a point of time.
 *     cost : The total cost in a particular time point.
 *     dynamicsViolation : The vector of dynamics violation.
 *     stateEqConstraint : An array of all state equality constraints.
 *     stateInputEqConstraint : An array of all state-input equality constraints.
 *     stateIneqConstraint : An array of all state inequality constraints.
 *     stateInputIneqConstraint : An array of all state-input inequality constraints.
 *     stateEqLagrangian : An array of state equality constraint terms handled by Lagrangian method.
 *     stateIneqLagrangian : An array of state inequality constraint terms handled by Lagrangian method.
 *     stateInputEqLagrangian : An array of state-input equality constraint terms handled by Lagrangian method.
 *     stateInputIneqLagrangian : An array of state-input inequality constraint terms handled by Lagrangian method.
 */
struct Metrics {
  // Cost
  scalar_t cost;

  // Dynamics violation
  vector_t dynamicsViolation;

  // Equality constraints
  vector_array_t stateEqConstraint;
  vector_array_t stateInputEqConstraint;

  // Inequality constraints
  vector_array_t stateIneqConstraint;
  vector_array_t stateInputIneqConstraint;

  // Lagrangians
  std::vector<LagrangianMetrics> stateEqLagrangian;
  std::vector<LagrangianMetrics> stateIneqLagrangian;
  std::vector<LagrangianMetrics> stateInputEqLagrangian;
  std::vector<LagrangianMetrics> stateInputIneqLagrangian;

    /**
   * @brief /** Exchanges the values of Metrics */.
   * @param [in,out] other: Other.
   */
  void swap(Metrics& other);

  /** Clears the value of the Metrics */
  void clear();

    /**
   * @brief /** Returns true if *this is approximately equal to other, within the precision determined by prec. */.
   * @param [in] other: Other.
   * @param [in] prec: Prec.
   * @return True on success.
   */
  bool isApprox(const Metrics& other, scalar_t prec = 1e-8) const;
};

/**
 * @brief /** Sums penalties of an array of LagrangianMetrics. */.
 * @param [in] metricsArray: Metrics Array.
 * @return Result value.
 */
inline scalar_t SumPenalties(const std::vector<LagrangianMetrics>& metricsArray) {
  scalar_t s = 0.0;
  /**
   * @brief For each.
   * @param [in] m: M.
   * @return Result value.
   */
  std::for_each(metricsArray.begin(), metricsArray.end(), [&s](const LagrangianMetrics& m) { s += m.penalty; });
  return s;
}

/**
 * @brief /** Computes the sum of squared norm of constraints of an array of LagrangianMetrics. */.
 * @param [in] metricsArray: Metrics Array.
 * @return Result value.
 */
inline scalar_t ConstraintsSquaredNorm(const std::vector<LagrangianMetrics>& metricsArray) {
  scalar_t s = 0.0;
  /**
   * @brief For each.
   * @param [in] m: M.
   * @return Result value.
   */
  std::for_each(metricsArray.begin(), metricsArray.end(), [&s](const LagrangianMetrics& m) { /**
  * @brief Squared Norm.
  * @return Result value.
  */
 s += m.constraint.squaredNorm(); });
  return s;
}

/**
 * @brief /** Computes the sum of squared norm of a vector of equality constraints violation. */.
 * @param [in] eqConstraint: Eq Constraint.
 * @return Result value.
 */
inline scalar_t GetEqConstraintsSSE(const vector_t& eqConstraint) {
  if (eqConstraint.size() == 0) {
    return 0.0;
  } else {
    /**
     * @brief Squared Norm.
     * @return Result value.
     */
    return eqConstraint.squaredNorm();
  }
}

/**
 * @brief /** Computes the sum of squared norm of a vector of inequality constraints violation. */.
 * @param [in] ineqConstraint: Ineq Constraint.
 * @return Result value.
 */
inline scalar_t GetIneqConstraintsSSE(const vector_t& ineqConstraint) {
  if (ineqConstraint.size() == 0) {
    return 0.0;
  } else {
    /**
     * @brief Cwise Min.
     * @return Result value.
     */
    return ineqConstraint.cwiseMin(0.0).squaredNorm();
  }
}

/**
 * @brief /** Computes the sum of squared norm of an array of equality constraints violation. */.
 * @param [in] eqConstraint: Eq Constraint.
 * @return Result value.
 */
inline scalar_t GetEqConstraintsSSE(const vector_array_t& eqConstraint) {
  scalar_t s = 0.0;
  /**
   * @brief For each.
   * @param [in] v: V.
   * @return Result value.
   */
  std::for_each(eqConstraint.begin(), eqConstraint.end(), [&s](const vector_t& v) { /**
  * @brief Get Eq Constraints SSE.
  * @param [in] v: V.
  * @return Result value.
  */
 s += GetEqConstraintsSSE(v); });
  return s;
}

/**
 * @brief /** Computes the sum of squared norm of an array of inequality constraints violation. */.
 * @param [in] ineqConstraint: Ineq Constraint.
 * @return Result value.
 */
inline scalar_t GetIneqConstraintsSSE(const vector_array_t& ineqConstraint) {
  scalar_t s = 0.0;
  /**
   * @brief For each.
   * @param [in] v: V.
   * @return Result value.
   */
  std::for_each(ineqConstraint.begin(), ineqConstraint.end(), [&s](const vector_t& v) { /**
  * @brief Get Ineq Constraints SSE.
  * @param [in] v: V.
  * @return Result value.
  */
 s += GetIneqConstraintsSSE(v); });
  return s;
}

/**
 * @brief /**.
 *
 * Serializes an array of LagrangianMetrics structures associated to an array of constraint terms.
 *
 * /
 * @param [in] termsLagrangianMetrics: Terms Lagrangian Metrics.
 * @return Serialized vector of the format : (..., termsMultiplier[i].penalty, termsMultiplier[i].constraint, ...).
 */
vector_t ToVector(const std::vector<LagrangianMetrics>& termsLagrangianMetrics);

/**
 * @brief /**.
 *
 * Serializes an array of constraint terms.
 *
 * /
 * @param [in] constraintArray: Constraint Array.
 * @return Serialized vector of the format : (...,  constraintArray[i], ...).
 */
vector_t ToVector(const vector_array_t& constraintArray);

/**
 * @brief /**.
 *
 * Gets the size of constraint terms.
 *
 * /
 * @param [in] constraintArray: Constraint Array.
 * @return An array of constraint terms size. It has the same size as the input array.
 */
size_array_t GetSizes(const vector_array_t& constraintArray);

/**
 * @brief /**.
 *
 * Gets the size of constraint Lagrangian terms.
 *
 * /
 * @param [in] termsLagrangianMetrics: Terms Lagrangian Metrics.
 * @return An array of constraint terms size. It has the same size as the input array.
 */
size_array_t GetSizes(const std::vector<LagrangianMetrics>& termsLagrangianMetrics);

/**
 * Deserializes the vector to an array of constraint terms.
 *
 * @param [in] termsSize: An array of constraint terms size. It as the same size as the output array.
 * @param [in] vec: Serialized array of constraint terms of the format :
 *                   (..., constraintArray[i], ...)
 * @return An array of constraint terms.
 */
vector_array_t ToConstraintArray(const size_array_t& termsSize, const vector_t& vec);

/**
 * Deserializes the vector to an array of LagrangianMetrics structures based on size of constraint terms.
 *
 * @param [in] termsSize: An array of constraint terms size. It as the same size as the output array.
 * @param [in] vec: Serialized array of LagrangianMetrics structures of the format :
 *                   (..., termsMultiplier[i].penalty, termsMultiplier[i].constraint, ...)
 * @return An array of LagrangianMetrics structures associated to an array of constraint terms
 */
std::vector<LagrangianMetrics> ToLagrangianMetrics(const size_array_t& termsSize, const vector_t& vec);

}  // namespace bolza

namespace bolza {
namespace LinearInterpolation {

/**
 * Linearly interpolates a trajectory of LagrangianMetrics.
 *
 * @param [in] indexAlpha: index and interpolation coefficient (alpha) pair.
 * @param [in] dataArray: A trajectory of LagrangianMetricsConstRef.
 * @return The interpolated LagrangianMetrics at indexAlpha.
 */
LagrangianMetrics Interpolate(const index_alpha_t& indexAlpha, const std::vector<LagrangianMetricsConstRef>& dataArray);

/**
 * Linearly interpolates a trajectory of Metrics.
 *
 * @param [in] indexAlpha: index and interpolation coefficient (alpha) pair.
 * @param [in] dataArray: A trajectory of Metrics.
 * @return The interpolated Metrics at indexAlpha.
 */
Metrics Interpolate(const index_alpha_t& indexAlpha, const std::vector<Metrics>& dataArray);

}  // namespace LinearInterpolation
}  // namespace bolza
