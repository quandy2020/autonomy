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

#include <iostream>

#include "bolza/core/misc/linear_algebra.hpp"
#include "bolza/core/model_data/model_data.hpp"

namespace bolza {

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
std::string CheckSize(const ModelData& data, int stateDim, int inputDim) {
  std::stringstream errorDescription;

  if (data.stateDim != stateDim) {
    errorDescription << "data.stateDim != " << stateDim << "\n";
  }
  if (data.inputDim != inputDim) {
    errorDescription << "data.inputDim != " << inputDim << "\n";
  }

  // dynamics
  if (data.dynamics.f.size() > 0) {
    errorDescription << CheckSize(stateDim, stateDim, inputDim, data.dynamics, "dynamics");

    if (data.dynamicsBias.size() != stateDim) {
      errorDescription << "dynamicsBias.size() != " << stateDim << "\n";
    }
  }

  // cost
  errorDescription << CheckSize(stateDim, inputDim, data.cost, "cost");

  // state equality constraints
  errorDescription << CheckSize(data.stateEqConstraint.f.size(), stateDim, 0, data.stateEqConstraint, "stateEqConstraint");

  // state-input equality constraints
  errorDescription << CheckSize(data.stateInputEqConstraint.f.size(), stateDim, inputDim, data.stateInputEqConstraint,
                                "stateInputEqConstraint");

  return errorDescription.str();
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
std::string CheckCostProperties(const ModelData& data) {
  std::stringstream errorDescription;

  errorDescription << CheckBeingPSD(data.cost, "cost");

  // check if R is invertible and its schur complement is PSD
  if (data.cost.dfduu.size() > 0) {
    const auto rcond = data.cost.dfduu.ldlt().rcond();
    if (rcond < Eigen::NumTraits<scalar_t>::epsilon()) {
      errorDescription << "Cost second derivative w.r.t. input is not invertible. It's reciprocal condition number is " +
                              std::to_string(rcond) + ".\n";
    } else {
      // check schur complement of R being PSD
      errorDescription << SchurComplementOfCostHessianIsPsd(data.cost);
    }
  }

  return errorDescription.str();
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
std::string SchurComplementOfCostHessianIsPsd(const ScalarFunctionQuadraticApproximation& cost) {
  if (cost.dfdxx.size() > 0 && cost.dfduu.size() > 0) {
    matrix_t UofUUT;
    LinearAlgebra::ComputeInverseMatrixUUT(cost.dfduu, UofUUT);
    const matrix_t UT_P = UofUUT.transpose() * cost.dfdux;
    matrix_t inputHessianSchurComplement = cost.dfdxx;
    inputHessianSchurComplement.noalias() -= UT_P.transpose() * UT_P;

    // check for being psd
    return CheckBeingPSD(inputHessianSchurComplement, "Schur complement of cost second derivative w.r.t. input");

  } else {
    return "Either cost.dfdxx or cost.dfduu are not set!";
  }
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
std::string CheckDynamicsProperties(const ModelData& data) {
  std::stringstream errorDescription;

  if (!data.dynamics.f.allFinite()) {
    errorDescription << "Dynamics is not finite.";
  }
  if (!data.dynamicsBias.allFinite()) {
    errorDescription << "Dynamics bias is not finite.";
  }
  if (!data.dynamics.dfdx.allFinite()) {
    errorDescription << "Dynamics derivative w.r.t. state is not finite.";
  }
  if (!data.dynamics.dfdu.allFinite()) {
    errorDescription << "Dynamics derivative w.r.t. input is not finite.";
  }

  return errorDescription.str();
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
std::string CheckControllability(const VectorFunctionLinearApproximation& dynamics) {
  const size_t stateDim = dynamics.dfdu.rows();
  const size_t inputDim = dynamics.dfdu.cols();

  // controllability matrix
  matrix_t ctrlMatrix(stateDim, inputDim * stateDim);
  ctrlMatrix.leftCols(inputDim) = dynamics.dfdu;
  for (size_t i = 1; i < stateDim; i++) {
    ctrlMatrix.middleCols(i * inputDim, inputDim).noalias() = dynamics.dfdx * ctrlMatrix.middleCols((i - 1) * inputDim, inputDim);
  }

  // controllability rank
  const size_t ctrlMatrixRank = LinearAlgebra::Rank(ctrlMatrix);

  std::stringstream errorDescription;
  if (ctrlMatrixRank < stateDim) {
    errorDescription << "Uncontrollable system: controllability matrix rank should be " << stateDim << " as opposed to " << ctrlMatrixRank
                     << "\n";
  }

  return errorDescription.str();
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
std::string CheckConstraintProperties(const ModelData& data) {
  std::stringstream errorDescription;

  if (data.stateEqConstraint.f.rows() > 0) {
    if (!data.stateEqConstraint.f.allFinite()) {
      errorDescription << "State-only constraint is not finite.\n";
    }
    if (!data.stateEqConstraint.dfdx.allFinite()) {
      errorDescription << "State-only constraint derivative w.r.t. state is not finite.\n";
    }
  }

  if (data.stateInputEqConstraint.f.rows() > 0) {
    const auto inputDim = data.stateInputEqConstraint.dfdu.cols();
    const auto numConstraints = data.stateInputEqConstraint.f.rows();

    if (!data.stateInputEqConstraint.f.allFinite()) {
      errorDescription << "Input-state constraint is not finite.\n";
    }
    if (!data.stateInputEqConstraint.dfdx.allFinite()) {
      errorDescription << "Input-state constraint derivative w.r.t. state is not finite.\n";
    }
    if (!data.stateInputEqConstraint.dfdu.allFinite()) {
      errorDescription << "Input-state constraint derivative w.r.t. input is not finite.\n";
    }
    if (numConstraints > inputDim) {
      errorDescription << "Number of active state-input equality constraints (a.k.a. " + std::to_string(numConstraints) +
                              ") should be less-equal to the input dimension (a.k.a. " + std::to_string(inputDim) + ").\n";
    }
    const size_t DmRank = LinearAlgebra::Rank(data.stateInputEqConstraint.dfdu);
    if (DmRank != numConstraints) {
      errorDescription << "Input-state constraint derivative w.r.t. input is not full-row rank. It's rank is " + std::to_string(DmRank) +
                              " while the expected rank is " + std::to_string(numConstraints) + ".\n";
    }
  }

  return errorDescription.str();
}

}  // namespace bolza
