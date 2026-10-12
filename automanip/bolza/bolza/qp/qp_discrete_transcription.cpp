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

#include <bolza/oc/approximate_model/linear_quadratic_approximator.hpp>

#include "bolza/qp/qp_discrete_transcription.hpp"

namespace bolza {
namespace qp_solver {

std::vector<LinearQuadraticStage> GetLinearQuadraticApproximation(OptimalControlProblem& optimalControProblem,
                                                                  const ContinuousTrajectory& nominalTrajectory) {
  // OCP check
  if (!optimalControProblem.equalityLagrangianPtr->empty() || !optimalControProblem.stateEqualityLagrangianPtr->empty()) {
    throw std::runtime_error("[getLinearQuadraticApproximation] equalityLagrangianPtr and stateEqualityLagrangianPtr should be empty!");
  }
  if (!optimalControProblem.inequalityLagrangianPtr->empty() || !optimalControProblem.stateInequalityLagrangianPtr->empty()) {
    throw std::runtime_error("[getLinearQuadraticApproximation] inequalityLagrangianPtr and stateInequalityLagrangianPtr should be empty!");
  }
  if (!optimalControProblem.finalEqualityLagrangianPtr->empty() || !optimalControProblem.finalInequalityLagrangianPtr->empty()) {
    throw std::runtime_error(
        "[getLinearQuadraticApproximation] finalEqualityLagrangianPtr and finalInequalityLagrangianPtr should be empty!");
  }

  if (nominalTrajectory.timeTrajectory.empty()) {
    return {};
  }

  auto& t = nominalTrajectory.timeTrajectory;
  auto& x = nominalTrajectory.stateTrajectory;
  auto& u = nominalTrajectory.inputTrajectory;
  const int N = t.size() - 1;

  // LinearQuadraticProblem with N+1 elements. Terminal stage lqp[N].dynamics is ignored.
  std::vector<LinearQuadraticStage> lqp;
  lqp.reserve(N + 1);
  for (int k = 0; k < N; ++k) {  // Intermediate stages
    lqp.emplace_back(ApproximateStage(optimalControProblem, {t[k], x[k], u[k]}, {t[k + 1], x[k + 1]}, k == 0));
  }

  auto modelData = ApproximateFinalLQ(optimalControProblem, t[N], x[N], MultiplierCollection());

  // checking the numerical properties
  const auto errSize = CheckSize(modelData, x[N].rows(), 0);
  if (!errSize.empty()) {
    throw std::runtime_error("[qp_solver::GetLinearQuadraticApproximation] Ill-posed problem at final time: " + std::to_string(t[N]) +
                             "\n" + errSize);
  }
  const std::string errProperties = CheckCostProperties(modelData) + CheckConstraintProperties(modelData);
  if (!errProperties.empty()) {
    throw std::runtime_error("[qp_solver::GetLinearQuadraticApproximation] Ill-posed problem at final time: " + std::to_string(t[N]) +
                             "\n" + errProperties);
  }

  lqp.emplace_back(std::move(modelData.cost), VectorFunctionLinearApproximation(), std::move(modelData.stateEqConstraint));

  return lqp;
}

LinearQuadraticStage ApproximateStage(OptimalControlProblem& optimalControProblem, TrajectoryRef start, StateTrajectoryRef end,
                                      bool isInitialTime) {
  const auto modelData = ApproximateIntermediateLQ(optimalControProblem, start.t, start.x, start.u, MultiplierCollection());

  // checking the numerical properties
  const auto errSize = CheckSize(modelData, start.x.rows(), start.u.rows());
  if (!errSize.empty()) {
    throw std::runtime_error("[[qp_solver::ApproximateStage] Ill-posed problem at intermediate time: " + std::to_string(start.t) + "\n" +
                             errSize);
  }
  const std::string errProperties =
      CheckDynamicsProperties(modelData) + CheckCostProperties(modelData) + CheckConstraintProperties(modelData);
  if (!errProperties.empty()) {
    throw std::runtime_error("[qp_solver::ApproximateStage] Ill-posed problem at intermediate time: " + std::to_string(start.t) + "\n" +
                             errProperties);
  }

  LinearQuadraticStage lqStage;
  const auto dt = end.t - start.t;

  lqStage.cost = modelData.cost;
  lqStage.cost *= dt;

  // Linearized Dynamics after discretization: x0[k+1] + dx[k+1] = A dx[k] + B du[k] + F(x0[k], u0[k])
  lqStage.dynamics = ApproximateDynamics(modelData, start, dt);
  // Adapt the offset to account for discretization and the nominal trajectory :
  // dx[k+1] = A dx[k] + B du[k] + F(x0[k], u0[k]) - x0[k+1]
  lqStage.dynamics.f -= end.x;

  lqStage.constraints = ApproximateConstraints(modelData, isInitialTime);

  return lqStage;
}

VectorFunctionLinearApproximation ApproximateDynamics(const ModelData& modelData, TrajectoryRef start, scalar_t dt) {
  // Forward Euler discretization
  // x[k+1] = x[k] + dt * dxdt[k]
  // x[k+1] = (x0[k] + dx[k]) + dt * dxdt[k]
  // x[k+1] = (x0[k] + dx[k]) + dt * (A_c dx[k] + B_c du[k] + b_c)
  // x[k+1] = (I + A_c * dt) dx[k] + (B_c * dt) du[k] + (b_c * dt + x0[k])
  const auto& continuousDynamics = modelData.dynamics;
  VectorFunctionLinearApproximation discreteDynamics;
  discreteDynamics.dfdx = continuousDynamics.dfdx * dt;
  discreteDynamics.dfdx.diagonal().array() += 1.0;
  discreteDynamics.dfdu = continuousDynamics.dfdu * dt;
  discreteDynamics.f = continuousDynamics.f * dt + start.x;
  return discreteDynamics;
}

VectorFunctionLinearApproximation ApproximateConstraints(const ModelData& modelData, bool isInitialTime) {
  VectorFunctionLinearApproximation constraintsApproximation;
  if (isInitialTime) {
    // only use stat-input constraints for initial time
    constraintsApproximation = std::move(modelData.stateInputEqConstraint);
  } else {
    // concatenate stat-input and state-only constraints
    const auto& stateInputConstraint = modelData.stateInputEqConstraint;
    const auto& stateConstraint = modelData.stateEqConstraint;
    const auto numStateInputConstraints = stateInputConstraint.f.size();
    const auto numStateConstraints = stateConstraint.f.size();
    const auto numStates = stateInputConstraint.dfdx.cols();
    const auto numInputs = stateInputConstraint.dfdu.cols();
    constraintsApproximation.resize(numStateConstraints + numStateInputConstraints, numStates, numInputs);
    constraintsApproximation.f.head(numStateInputConstraints) = stateInputConstraint.f;
    constraintsApproximation.dfdx.topRows(numStateInputConstraints) = stateInputConstraint.dfdx;
    constraintsApproximation.dfdu.topRows(numStateInputConstraints) = stateInputConstraint.dfdu;
    constraintsApproximation.f.tail(numStateConstraints) = stateConstraint.f;
    constraintsApproximation.dfdx.bottomRows(numStateConstraints) = stateConstraint.dfdx;
    constraintsApproximation.dfdu.bottomRows(numStateConstraints).setZero();
  }
  return constraintsApproximation;
}

}  // namespace qp_solver
}  // namespace bolza
