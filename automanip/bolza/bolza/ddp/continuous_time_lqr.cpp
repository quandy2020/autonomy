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

#include "bolza/ddp/continuous_time_lqr.hpp"

#include <bolza/core/misc/linear_algebra.hpp>
#include <bolza/oc/approximate_model/linear_quadratic_approximator.hpp>

namespace bolza {
namespace continuous_time_lqr {

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
solution solve(OptimalControlProblem& problem, scalar_t time, const vector_t& state, const vector_t& input, const Settings& settings) {
  // OCP check
  if (!problem.equalityLagrangianPtr->empty() || !problem.stateEqualityLagrangianPtr->empty()) {
    throw std::runtime_error("[getLinearQuadraticApproximation] equalityLagrangianPtr and stateEqualityLagrangianPtr should be empty!");
  }
  if (!problem.inequalityLagrangianPtr->empty() || !problem.stateInequalityLagrangianPtr->empty()) {
    throw std::runtime_error("[getLinearQuadraticApproximation] inequalityLagrangianPtr and stateInequalityLagrangianPtr should be empty!");
  }

  const size_t stateDim = state.size();

  // --- Form the Linear quadratic approximation ---
  // Obtain model data at the provided reference
  const auto modelData = ApproximateIntermediateLQ(problem, time, state, input, MultiplierCollection());

  // checking the numerical properties
  if (settings.checkNumericalCharacteristics) {
    const std::string err = CheckDynamicsProperties(modelData) + CheckCostProperties(modelData) + CheckConstraintProperties(modelData);
    if (!err.empty()) {
      throw std::runtime_error("[continuous_time_lqr::solve] Ill-posed problem at intermediate time: " + std::to_string(time) + "\n" + err);
    }
  }

  // --- Contstruct hamiltonian ---
  // Compute terms containing inv(R)
  matrix_t RinvU;
  LinearAlgebra::ComputeInverseMatrixUUT(modelData.cost.dfduu, RinvU);  // Rinv = RinvU * RinvU.transpose()
  const matrix_t B_RinvU = modelData.dynamics.dfdu * RinvU;
  matrix_t PT_RinvU = modelData.cost.dfdux.transpose() * RinvU;
  const matrix_t B_Rinv_BT = B_RinvU * B_RinvU.transpose();
  const matrix_t B_Rinv_P = B_RinvU * PT_RinvU.transpose();
  const matrix_t PT_Rinv_P = PT_RinvU * PT_RinvU.transpose();

  // Correct for state-input matrix
  const matrix_t A = modelData.dynamics.dfdx - B_Rinv_P;
  const matrix_t Q = modelData.cost.dfdxx - PT_Rinv_P;

  // The permuted Hamiltonian is the initial point of the iterative algorithm
  matrix_t W = (matrix_t(2 * stateDim, 2 * stateDim) << -Q, -A.transpose(), -A, B_Rinv_BT).finished();

  // --- Solve CARE ---

  // Temporary variables used in the iteration
  matrix_t Winv(2 * stateDim, 2 * stateDim);
  matrix_t dW(2 * stateDim, 2 * stateDim);
  matrix_t JWinvJ(2 * stateDim, 2 * stateDim);

  const scalar_t exponent = 0.5 / stateDim;
  scalar_t Wnorm;
  size_t iter = 0;
  do {
    // Store scale of current iteration
    Wnorm = W.norm();

    // Prepare update: W(k+1) = 1/(2c) * (W(k) + c^2 * J *  inv(W(k)) * J)
    // W(k+1) - W(k) = c1 * W(k) + c2 *  J *  inv(W(k)) * J
    const scalar_t c = std::pow(std::abs(W.determinant()), exponent);
    const scalar_t c1 = 0.5 / c - 1.0;
    const scalar_t c2 = 0.5 * c;
    Winv = W.partialPivLu().inverse();
    JWinvJ << -Winv.bottomRightCorner(stateDim, stateDim), Winv.bottomLeftCorner(stateDim, stateDim),
        Winv.topRightCorner(stateDim, stateDim), -Winv.topLeftCorner(stateDim, stateDim);  // Implement J * Winv * J, with J = [0, I; -I 0]
    dW = c1 * W + c2 * JWinvJ;

    // Apply update
    W += dW;
    iter++;
  } while ((dW.norm() / Wnorm) > settings.tolerance && iter < settings.maxIter);

  // Solve for value function through system of equations M * S = N
  matrix_t lhsM(2 * stateDim, stateDim);
  lhsM << W.bottomRightCorner(stateDim, stateDim), W.topRightCorner(stateDim, stateDim) + matrix_t::Identity(stateDim, stateDim);
  matrix_t rhsN(2 * stateDim, stateDim);
  rhsN << matrix_t::Identity(stateDim, stateDim) - W.bottomLeftCorner(stateDim, stateDim), -W.topLeftCorner(stateDim, stateDim);
  const matrix_t S = lhsM.colPivHouseholderQr().solve(rhsN);

  // Compute feedback gains
  PT_RinvU.noalias() += S.transpose() * B_RinvU;
  const matrix_t K = -RinvU * PT_RinvU.transpose();

  return {K, S};
}

}  // namespace continuous_time_lqr
}  // namespace bolza
