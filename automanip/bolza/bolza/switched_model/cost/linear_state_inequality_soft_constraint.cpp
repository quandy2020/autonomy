//
// Created by rgrandia on 26.07.21.
//

#include "bolza/switched_model/cost/linear_state_inequality_softconstraint.hpp"

namespace switched_model {

scalar_t GetValue(const LinearStateInequalitySoftConstraint& constraints, const vector_t& f) {
  scalar_t Cost(0.0);
  for (int j = 0; j < constraints.h.size(); ++j) {  // Loop through all faces of the constraint
    cost += constraints.penalty->getValue(0.0, constraints.h(j));
  }

  return cost;
}

scalar_t GetValue(const SingleLinearStateInequalitySoftConstraint& constraint, const vector_t& f) {
  return constraint.penalty->getValue(0.0, constraint.h);
}

ScalarFunctionQuadraticApproximation GetQuadraticApproximation(const LinearStateInequalitySoftConstraint& constraints, const vector_t& f,
                                                               const matrix_t& dfdx) {
  ScalarFunctionQuadraticApproximation cost;

  const auto& h = constraints.h;

  cost.f = 0.0;
  for (int j = 0; j < h.size(); ++j) {  // Loop through all faces of the constraint
    cost.f += constraints.penalty->getValue(0.0, h(j));
  }

  const auto penaltyDerivatives = h.unaryExpr([&](scalar_t hi) { return constraints.penalty->getDerivative(0.0, hi); });
  const vector_t taskSpaceDerivative = constraints.A.transpose() * penaltyDerivatives;
  cost.dfdx.noalias() = dfdx.transpose() * taskSpaceDerivative;

  const auto penaltySecondDerivatives = h.unaryExpr([&](scalar_t hi) { return constraints.penalty->getSecondDerivative(0.0, hi); });
  const matrix_t scaledConstraint = penaltySecondDerivatives.asDiagonal() * constraints.A;
  const matrix_t taskSpaceSecondDerivative = constraints.A.transpose() * scaledConstraint;
  const matrix_t scaledJacobian = taskSpaceSecondDerivative * dfdx;
  cost.dfdxx.noalias() = dfdx.transpose() * scaledJacobian;

  return cost;
}

ScalarFunctionQuadraticApproximation GetQuadraticApproximation(const SingleLinearStateInequalitySoftConstraint& constraint,
                                                               const vector_t& f, const matrix_t& dfdx) {
  ScalarFunctionQuadraticApproximation cost;

  const auto& h = constraint.h;

  cost.f = constraint.penalty->getValue(0.0, constraint.h);

  const Eigen::Matrix<scalar_t, 3, 1> taskSpaceDerivative = constraint.A.transpose() * constraint.penalty->getDerivative(0.0, h);
  cost.dfdx.noalias() = dfdx.transpose() * taskSpaceDerivative;

  const matrix_t jacobian = constraint.A * dfdx;
  const matrix_t scaledJacobian = constraint.penalty->getSecondDerivative(0.0, h) * jacobian;
  cost.dfdxx.noalias() = jacobian.transpose() * scaledJacobian;

  return cost;
}

}  // namespace switched_model
