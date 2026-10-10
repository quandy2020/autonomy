/*
 * Copyright 2026 Automanip contributors duyongquan (quandy2020@126.com)
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

#include "automanip/ipm/lq_solver.hpp"

#include <iostream>
#include <stdexcept>

namespace automanip {
namespace ipm {

std::ostream& operator<<(std::ostream& stream, const LqSettings& settings) {
  stream << " #### LQ Riccati settings:\n";
  stream << " #### hessianReg: " << settings.hessianReg << "\n";
  return stream;
}

}  // namespace ipm

namespace {

matrix_t Symmetrize(matrix_t value) {
  value = (value + value.transpose()).eval() * 0.5;
  return value;
}

bool FactorControlHessian(matrix_t& controlHessian, scalar_t regularization, Eigen::LDLT<matrix_t>* factorization) {
  factorization->compute(controlHessian);
  if (factorization->info() == Eigen::Success && factorization->isPositive()) {
    return true;
  }
  controlHessian.diagonal().array() += regularization;
  factorization->compute(controlHessian);
  return factorization->info() == Eigen::Success && factorization->isPositive();
}

bool SolveEqualityStage(matrix_t controlHessian, const matrix_t& crossHessian, const vector_t& controlGradient, matrix_t constraintState,
                        matrix_t constraintInput, const vector_t& constraintValue, scalar_t regularization, matrix_t* feedback,
                        vector_t* feedforward) {
  const int numInputs = controlHessian.rows();
  const int numConstraints = constraintValue.size();
  const int numStates = crossHessian.cols();
  if (constraintState.size() == 0) {
    constraintState = matrix_t::Zero(numConstraints, numStates);
  }
  if (constraintInput.size() == 0) {
    constraintInput = matrix_t::Zero(numConstraints, numInputs);
  }

  auto factor = [&](const matrix_t& hessian) {
    matrix_t kkt = matrix_t::Zero(numInputs + numConstraints, numInputs + numConstraints);
    kkt.topLeftCorner(numInputs, numInputs) = hessian;
    kkt.topRightCorner(numInputs, numConstraints) = constraintInput.transpose();
    kkt.bottomLeftCorner(numConstraints, numInputs) = constraintInput;
    matrix_t stateRightHandSide(numInputs + numConstraints, numStates);
    stateRightHandSide.topRows(numInputs) = -crossHessian;
    stateRightHandSide.bottomRows(numConstraints) = -constraintState;
    vector_t affineRightHandSide(numInputs + numConstraints);
    affineRightHandSide.head(numInputs) = -controlGradient;
    affineRightHandSide.tail(numConstraints) = -constraintValue;
    const Eigen::FullPivLU<matrix_t> factorization(kkt);
    if (!factorization.isInvertible()) {
      return false;
    }
    const matrix_t stateSolution = factorization.solve(stateRightHandSide);
    const vector_t affineSolution = factorization.solve(affineRightHandSide);
    *feedback = stateSolution.topRows(numInputs);
    *feedforward = affineSolution.head(numInputs);
    return feedback->allFinite() && feedforward->allFinite();
  };

  if (factor(controlHessian)) {
    return true;
  }
  controlHessian.diagonal().array() += regularization;
  return factor(controlHessian);
}

}  // namespace

LqSolver::LqSolver(OcpSize ocpSize, const ipm::LqSettings& settings) : settings_(settings), ocpSize_(std::move(ocpSize)) {}

void LqSolver::resize(OcpSize ocpSize) {
  ocpSize_ = std::move(ocpSize);
  solved_ = false;
}

void LqSolver::verifySizes(const vector_t& x0, const std::vector<VectorFunctionLinearApproximation>& dynamics,
                           const std::vector<ScalarFunctionQuadraticApproximation>& cost) const {
  if (static_cast<int>(dynamics.size()) != ocpSize_.numStages) {
    throw std::runtime_error("[LqSolver] Inconsistent size of dynamics: " + std::to_string(dynamics.size()) + " with " +
                             std::to_string(ocpSize_.numStages) + " number of stages.");
  }
  if (static_cast<int>(cost.size()) != ocpSize_.numStages + 1) {
    throw std::runtime_error("[LqSolver] Inconsistent size of cost: " + std::to_string(cost.size()) + " with " +
                             std::to_string(ocpSize_.numStages + 1) + " nodes.");
  }
  if (!dynamics.empty() && x0.size() != dynamics.front().dfdx.cols()) {
    throw std::runtime_error("[LqSolver] Initial state size does not match the first dynamics Jacobian.");
  }
}

LqStatus LqSolver::solve(const vector_t& x0, std::vector<VectorFunctionLinearApproximation>& dynamics,
                         std::vector<ScalarFunctionQuadraticApproximation>& cost,
                         std::vector<VectorFunctionLinearApproximation>* constraints, vector_array_t& stateTrajectory,
                         vector_array_t& inputTrajectory, bool verbose) {
  solved_ = false;
  verifySizes(x0, dynamics, cost);
  if (constraints != nullptr && static_cast<int>(constraints->size()) != static_cast<int>(dynamics.size()) + 1 && !constraints->empty()) {
    throw std::runtime_error("[LqSolver] Equality constraints must have one entry per node.");
  }

  const int numStages = static_cast<int>(dynamics.size());
  costToGo_.assign(numStages + 1, ScalarFunctionQuadraticApproximation());
  feedback_.assign(numStages, matrix_t());
  feedforward_.assign(numStages, vector_t());

  if (numStages == 0) {
    stateTrajectory = {x0};
    inputTrajectory.clear();
    if (!cost.empty()) {
      costToGo_[0].dfdxx = Symmetrize(cost[0].dfdxx);
      costToGo_[0].dfdx = cost[0].dfdx;
    }
    solved_ = x0.allFinite();
    return solved_ ? LqStatus::SUCCESS : LqStatus::NAN_SOL;
  }

  if (constraints != nullptr && !constraints->empty() && constraints->back().f.size() > 0) {
    if (verbose) {
      std::cerr << "[LqSolver] Terminal equalities are not part of the Riccati stage." << std::endl;
    }
    return LqStatus::NAN_SOL;
  }

  costToGo_[numStages].dfdxx = Symmetrize(cost[numStages].dfdxx);
  costToGo_[numStages].dfdx = cost[numStages].dfdx;

  for (int stage = numStages - 1; stage >= 0; --stage) {
    const matrix_t& dynamicsState = dynamics[stage].dfdx;
    const matrix_t& dynamicsInput = dynamics[stage].dfdu;
    const vector_t& dynamicsAffine = dynamics[stage].f;
    const matrix_t& costState = cost[stage].dfdxx;
    const vector_t& costStateGradient = cost[stage].dfdx;
    const matrix_t& costToGoState = costToGo_[stage + 1].dfdxx;
    const vector_t& costToGoGradient = costToGo_[stage + 1].dfdx;
    const int numInputs = dynamicsInput.cols();

    const vector_t affineCostToGo = costToGoState * dynamicsAffine + costToGoGradient;
    matrix_t stateHessian = costState + dynamicsState.transpose() * costToGoState * dynamicsState;
    vector_t stateGradient = costStateGradient + dynamicsState.transpose() * affineCostToGo;

    const bool hasEquality = constraints != nullptr && (*constraints)[stage].f.size() > 0;
    if (numInputs == 0) {
      if (hasEquality) {
        if (verbose) {
          std::cerr << "[LqSolver] Stage " << stage << " has equalities but no inputs." << std::endl;
        }
        return LqStatus::NAN_SOL;
      }
      costToGo_[stage].dfdxx = Symmetrize(std::move(stateHessian));
      costToGo_[stage].dfdx = std::move(stateGradient);
      feedback_[stage].resize(0, dynamicsState.cols());
      feedforward_[stage].resize(0);
      continue;
    }

    matrix_t controlHessian = cost[stage].dfduu + dynamicsInput.transpose() * costToGoState * dynamicsInput;
    matrix_t crossHessian = cost[stage].dfdux;
    if (crossHessian.size() == 0) {
      crossHessian = matrix_t::Zero(numInputs, dynamicsState.cols());
    }
    crossHessian.noalias() += dynamicsInput.transpose() * costToGoState * dynamicsState;
    vector_t controlGradient = cost[stage].dfdu + dynamicsInput.transpose() * affineCostToGo;

    if (hasEquality) {
      const auto& constraint = (*constraints)[stage];
      if (!SolveEqualityStage(controlHessian, crossHessian, controlGradient, constraint.dfdx, constraint.dfdu, constraint.f,
                              settings_.hessianReg, &feedback_[stage], &feedforward_[stage])) {
        if (verbose) {
          std::cerr << "[LqSolver] Equality KKT system is singular at stage " << stage << std::endl;
        }
        return LqStatus::NAN_SOL;
      }
    } else {
      Eigen::LDLT<matrix_t> factorization;
      if (!FactorControlHessian(controlHessian, settings_.hessianReg, &factorization)) {
        if (verbose) {
          std::cerr << "[LqSolver] Control Hessian is not positive definite at stage " << stage << std::endl;
        }
        return LqStatus::NAN_SOL;
      }
      feedback_[stage] = -factorization.solve(crossHessian);
      feedforward_[stage] = -factorization.solve(controlGradient);
    }
    stateHessian.noalias() += crossHessian.transpose() * feedback_[stage];
    stateGradient.noalias() += crossHessian.transpose() * feedforward_[stage];
    costToGo_[stage].dfdxx = Symmetrize(std::move(stateHessian));
    costToGo_[stage].dfdx = std::move(stateGradient);
  }

  stateTrajectory.resize(numStages + 1);
  inputTrajectory.resize(numStages);
  stateTrajectory[0] = x0;
  for (int stage = 0; stage < numStages; ++stage) {
    if (feedback_[stage].size() == 0) {
      inputTrajectory[stage].resize(0);
    } else {
      inputTrajectory[stage] = feedback_[stage] * stateTrajectory[stage] + feedforward_[stage];
    }
    stateTrajectory[stage + 1] = dynamics[stage].f;
    stateTrajectory[stage + 1].noalias() += dynamics[stage].dfdx * stateTrajectory[stage];
    if (dynamics[stage].dfdu.size() > 0) {
      stateTrajectory[stage + 1].noalias() += dynamics[stage].dfdu * inputTrajectory[stage];
    }
    if (!stateTrajectory[stage + 1].allFinite() || !inputTrajectory[stage].allFinite()) {
      if (verbose) {
        std::cerr << "[LqSolver] Non-finite trajectory at stage " << stage << std::endl;
      }
      return LqStatus::NAN_SOL;
    }
  }

  solved_ = true;
  if (verbose) {
    std::cerr << "[LqSolver] Riccati recursion solved " << numStages << " stages." << std::endl;
  }
  return LqStatus::SUCCESS;
}

std::vector<ScalarFunctionQuadraticApproximation> LqSolver::getRiccatiCostToGo(const VectorFunctionLinearApproximation&,
                                                                               const ScalarFunctionQuadraticApproximation&) const {
  if (!solved_) {
    throw std::runtime_error("[LqSolver] getRiccatiCostToGo() requires a successful solve().");
  }
  return costToGo_;
}

matrix_array_t LqSolver::getRiccatiFeedback(const VectorFunctionLinearApproximation&, const ScalarFunctionQuadraticApproximation&) const {
  if (!solved_) {
    throw std::runtime_error("[LqSolver] getRiccatiFeedback() requires a successful solve().");
  }
  return feedback_;
}

vector_array_t LqSolver::getRiccatiFeedforward(const VectorFunctionLinearApproximation&,
                                               const ScalarFunctionQuadraticApproximation&) const {
  if (!solved_) {
    throw std::runtime_error("[LqSolver] getRiccatiFeedforward() requires a successful solve().");
  }
  return feedforward_;
}

}  // namespace automanip
