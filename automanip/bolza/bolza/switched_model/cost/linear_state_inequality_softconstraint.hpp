//
// Created by rgrandia on 26.07.21.
//

#pragma once

/**
 * @file linear_state_inequality_softconstraint.hpp
 * @brief Linear inequality constraint in task space: A * f(x) + b >= 0.
 */


#include <bolza/switched_model/core/switched_model.hpp>

#include <bolza/core/penalties/penalties/penalty_base.hpp>

namespace switched_model {
/**
 * Linear inequality constraint in task space: A * f(x) + b >= 0
 * h(x) = A * f(x) + b
 * together with the penalty function to apply to this constraint
 */
struct LinearStateInequalitySoftConstraint {
  Eigen::Matrix<scalar_t, -1, 3> A;
  vector_t h;
  const bolza::PenaltyBase* penalty;
};

/// Specialization where there is only 1 constraint
struct SingleLinearStateInequalitySoftConstraint {
  Eigen::Matrix<scalar_t, 1, 3> A;
  scalar_t h;
  const bolza::PenaltyBase* penalty;
};

/**
 * @brief Get Value.
 * @param [in] constraints: Constraints.
 * @param [in] f: F.
 * @return Result value.
 */
scalar_t GetValue(const LinearStateInequalitySoftConstraint& constraints, const vector_t& f);

/**
 * @brief Get Value.
 * @param [in] constraints: Constraints.
 * @param [in] f: F.
 * @return Result value.
 */
scalar_t GetValue(const SingleLinearStateInequalitySoftConstraint& constraints, const vector_t& f);

/**
 * @brief Get Quadratic Approximation.
 * @param [in] constraints: Constraints.
 * @param [in] f: F.
 * @param [in] dfdx: Dfdx.
 * @return Result value.
 */
ScalarFunctionQuadraticApproximation GetQuadraticApproximation(const LinearStateInequalitySoftConstraint& constraints, const vector_t& f,
                                                               const matrix_t& dfdx);

/**
 * @brief Get Quadratic Approximation.
 * @param [in] constraint: Constraint.
 * @param [in] f: F.
 * @param [in] dfdx: Dfdx.
 * @return Result value.
 */
ScalarFunctionQuadraticApproximation GetQuadraticApproximation(const SingleLinearStateInequalitySoftConstraint& constraint,
                                                               const vector_t& f, const matrix_t& dfdx);

}  // namespace switched_model
