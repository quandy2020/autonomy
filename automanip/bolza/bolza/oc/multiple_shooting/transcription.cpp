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

#include "bolza/oc/multiple_shooting/transcription.hpp"

#include <bolza/core/misc/linear_algebra.hpp>

#include "bolza/oc/approximate_model/change_of_input_variables.hpp"
#include "bolza/oc/approximate_model/linear_quadratic_approximator.hpp"

namespace bolza {
namespace multiple_shooting {

Transcription SetupIntermediateNode(OptimalControlProblem& optimalControlProblem, DynamicsSensitivityDiscretizer& sensitivityDiscretizer,
                                    scalar_t t, scalar_t dt, const vector_t& x, const vector_t& x_next, const vector_t& u) {
  // Results and short-hand notation
  Transcription transcription;
  auto& cost = transcription.cost;
  auto& dynamics = transcription.dynamics;
  auto& constraintsSize = transcription.constraintsSize;
  auto& stateEqConstraints = transcription.stateEqConstraints;
  auto& stateInputEqConstraints = transcription.stateInputEqConstraints;
  auto& stateIneqConstraints = transcription.stateIneqConstraints;
  auto& stateInputIneqConstraints = transcription.stateInputIneqConstraints;

  // Dynamics
  // Discretization returns x_{k+1} = A_{k} * dx_{k} + B_{k} * du_{k} + b_{k}
  dynamics = sensitivityDiscretizer(*optimalControlProblem.dynamicsPtr, t, x, u, dt);
  dynamics.f -= x_next;  // make it dx_{k+1} = ...

  // Precomputation for other terms
  constexpr auto request = Request::Cost + Request::SoftConstraint + Request::Constraint + Request::Approximation;
  optimalControlProblem.preComputationPtr->Request(request, t, x, u);

  // Costs: Approximate the integral with forward euler
  cost = ApproximateCost(optimalControlProblem, t, x, u);
  cost *= dt;

  // State equality constraints
  if (!optimalControlProblem.stateEqualityConstraintPtr->empty()) {
    constraintsSize.stateEq = optimalControlProblem.stateEqualityConstraintPtr->GetTermsSize(t);
    stateEqConstraints =
        optimalControlProblem.stateEqualityConstraintPtr->GetLinearApproximation(t, x, *optimalControlProblem.preComputationPtr);
  }

  // State-input equality constraints
  if (!optimalControlProblem.equalityConstraintPtr->empty()) {
    constraintsSize.stateInputEq = optimalControlProblem.equalityConstraintPtr->GetTermsSize(t);
    stateInputEqConstraints =
        optimalControlProblem.equalityConstraintPtr->GetLinearApproximation(t, x, u, *optimalControlProblem.preComputationPtr);
  }

  // State inequality constraints.
  if (!optimalControlProblem.stateInequalityConstraintPtr->empty()) {
    constraintsSize.stateIneq = optimalControlProblem.stateInequalityConstraintPtr->GetTermsSize(t);
    stateIneqConstraints =
        optimalControlProblem.stateInequalityConstraintPtr->GetLinearApproximation(t, x, *optimalControlProblem.preComputationPtr);
  }

  // State-input inequality constraints.
  if (!optimalControlProblem.inequalityConstraintPtr->empty()) {
    constraintsSize.stateInputIneq = optimalControlProblem.inequalityConstraintPtr->GetTermsSize(t);
    stateInputIneqConstraints =
        optimalControlProblem.inequalityConstraintPtr->GetLinearApproximation(t, x, u, *optimalControlProblem.preComputationPtr);
  }

  return transcription;
}

void ProjectTranscription(Transcription& transcription, bool extractProjectionMultiplier) {
  auto& cost = transcription.cost;
  auto& dynamics = transcription.dynamics;
  auto& stateInputEqConstraints = transcription.stateInputEqConstraints;
  auto& stateInputIneqConstraints = transcription.stateInputIneqConstraints;
  auto& projection = transcription.constraintsProjection;
  auto& projectionMultiplierCoefficients = transcription.projectionMultiplierCoefficients;

  if (stateInputEqConstraints.f.size() > 0) {
    // Projection stored instead of constraint, // TODO: benchmark between lu and qr method. LU seems slightly faster.
    if (extractProjectionMultiplier) {
      matrix_t constraintPseudoInverse;
      std::tie(projection, constraintPseudoInverse) = LinearAlgebra::QrConstraintProjection(stateInputEqConstraints);
      projectionMultiplierCoefficients.compute(cost, dynamics, projection, constraintPseudoInverse);
    } else {
      projection = LinearAlgebra::LuConstraintProjection(stateInputEqConstraints).first;
      projectionMultiplierCoefficients = ProjectionMultiplierCoefficients();
    }
    stateInputEqConstraints = VectorFunctionLinearApproximation();

    // Adapt dynamics, cost, and state-input inequality constraints
    ChangeOfInputVariables(dynamics, projection.dfdu, projection.dfdx, projection.f);
    ChangeOfInputVariables(cost, projection.dfdu, projection.dfdx, projection.f);
    if (stateInputIneqConstraints.f.size() > 0) {
      ChangeOfInputVariables(stateInputIneqConstraints, projection.dfdu, projection.dfdx, projection.f);
    }
  }
}

TerminalTranscription SetupTerminalNode(OptimalControlProblem& optimalControlProblem, scalar_t t, const vector_t& x) {
  // Results and short-hand notation
  TerminalTranscription transcription;
  auto& cost = transcription.cost;
  auto& constraintsSize = transcription.constraintsSize;
  auto& eqConstraints = transcription.eqConstraints;
  auto& ineqConstraints = transcription.ineqConstraints;

  constexpr auto request = Request::Cost + Request::SoftConstraint + Request::Approximation;
  optimalControlProblem.preComputationPtr->RequestFinal(request, t, x);

  // Costs
  cost = ApproximateFinalCost(optimalControlProblem, t, x);

  // State equality constraints.
  if (!optimalControlProblem.finalEqualityConstraintPtr->empty()) {
    constraintsSize.stateEq = optimalControlProblem.finalEqualityConstraintPtr->GetTermsSize(t);
    eqConstraints =
        optimalControlProblem.finalEqualityConstraintPtr->GetLinearApproximation(t, x, *optimalControlProblem.preComputationPtr);
  }

  // State inequality constraints.
  if (!optimalControlProblem.finalInequalityConstraintPtr->empty()) {
    constraintsSize.stateIneq = optimalControlProblem.finalInequalityConstraintPtr->GetTermsSize(t);
    ineqConstraints =
        optimalControlProblem.finalInequalityConstraintPtr->GetLinearApproximation(t, x, *optimalControlProblem.preComputationPtr);
  }

  return transcription;
}

EventTranscription SetupEventNode(OptimalControlProblem& optimalControlProblem, scalar_t t, const vector_t& x, const vector_t& x_next) {
  // Results and short-hand notation
  EventTranscription transcription;
  auto& cost = transcription.cost;
  auto& dynamics = transcription.dynamics;
  auto& constraintsSize = transcription.constraintsSize;
  auto& eqConstraints = transcription.eqConstraints;
  auto& ineqConstraints = transcription.ineqConstraints;

  constexpr auto request = Request::Cost + Request::SoftConstraint + Request::Dynamics + Request::Approximation;
  optimalControlProblem.preComputationPtr->RequestPreJump(request, t, x);

  // Dynamics
  // jump map returns // x_{k+1} = A_{k} * dx_{k} + b_{k}
  dynamics = optimalControlProblem.dynamicsPtr->JumpMapLinearApproximation(t, x);
  dynamics.f -= x_next;                // make it dx_{k+1} = ...
  dynamics.dfdu.setZero(x.size(), 0);  // Overwrite derivative that shouldn't exist.

  // Costs
  cost = ApproximateEventCost(optimalControlProblem, t, x);

  // State equality constraints.
  if (!optimalControlProblem.preJumpEqualityConstraintPtr->empty()) {
    constraintsSize.stateEq = optimalControlProblem.preJumpEqualityConstraintPtr->GetTermsSize(t);
    eqConstraints =
        optimalControlProblem.preJumpEqualityConstraintPtr->GetLinearApproximation(t, x, *optimalControlProblem.preComputationPtr);
  }

  // State inequality constraints.
  if (!optimalControlProblem.preJumpInequalityConstraintPtr->empty()) {
    constraintsSize.stateIneq = optimalControlProblem.preJumpInequalityConstraintPtr->GetTermsSize(t);
    ineqConstraints =
        optimalControlProblem.preJumpInequalityConstraintPtr->GetLinearApproximation(t, x, *optimalControlProblem.preComputationPtr);
  }

  return transcription;
}

}  // namespace multiple_shooting
}  // namespace bolza
