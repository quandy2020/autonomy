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

#include "bolza/slp/helpers.hpp"

#include <atomic>
#include <numeric>

namespace {
int GetNumDecisionVariables(const bolza::OcpSize& ocpSize) {
  return std::accumulate(ocpSize.numInputs.begin(), ocpSize.numInputs.end(),
                         std::accumulate(ocpSize.numStates.begin() + 1, ocpSize.numStates.end(), (int)0));
}

int GetNumDynamicsConstraints(const bolza::OcpSize& ocpSize) {
  return std::accumulate(ocpSize.numStates.begin() + 1, ocpSize.numStates.end(), (int)0);
}

int getNumGeneralEqualityConstraints(const bolza::OcpSize& ocpSize) {
  return std::accumulate(ocpSize.numIneqConstraints.begin(), ocpSize.numIneqConstraints.end(), (int)0);
}
}  // anonymous namespace

namespace bolza {
namespace slp {

scalar_t HessianEigenvaluesUpperBound(const OcpSize& ocpSize, const std::vector<ScalarFunctionQuadraticApproximation>& cost) {
  const vector_t rowwiseAbsSumH = HessianAbsRowSum(ocpSize, cost);
  return rowwiseAbsSumH.maxCoeff();
}

scalar_t GGTEigenvaluesUpperBound(ThreadPool& threadPool, const OcpSize& ocpSize,
                                  const std::vector<VectorFunctionLinearApproximation>& dynamics,
                                  const std::vector<VectorFunctionLinearApproximation>* constraintsPtr,
                                  const vector_array_t* scalingVectorsPtr) {
  const vector_t rowwiseAbsSumGGT = GGTAbsRowSumInParallel(threadPool, ocpSize, dynamics, constraintsPtr, scalingVectorsPtr);
  return rowwiseAbsSumGGT.maxCoeff();
}

vector_t HessianAbsRowSum(const OcpSize& ocpSize, const std::vector<ScalarFunctionQuadraticApproximation>& cost) {
  const int N = ocpSize.numStages;
  const int nu_0 = ocpSize.numInputs[0];
  vector_t res(GetNumDecisionVariables(ocpSize));
  res.head(nu_0) = cost[0].dfduu.cwiseAbs().rowwise().sum();

  int curRow = nu_0;
  for (int k = 1; k < N; k++) {
    const int nx_k = ocpSize.numStates[k];
    const int nu_k = ocpSize.numInputs[k];
    if (cost[k].dfdxx.size() != 0) {
      res.segment(curRow, nx_k) = cost[k].dfdxx.cwiseAbs().rowwise().sum();
    }
    if (cost[k].dfdux.size() != 0) {
      res.segment(curRow, nx_k) += cost[k].dfdux.transpose().cwiseAbs().rowwise().sum();
    }
    if (cost[k].dfdux.size() != 0) {
      res.segment(curRow + nx_k, nu_k) = cost[k].dfdux.cwiseAbs().rowwise().sum();
    }
    if (cost[k].dfduu.size() != 0) {
      res.segment(curRow + nx_k, nu_k) += cost[k].dfduu.cwiseAbs().rowwise().sum();
    }
    curRow += nx_k + nu_k;
  }
  const int nx_N = ocpSize.numStates[N];
  res.tail(nx_N) = cost[N].dfdxx.cwiseAbs().rowwise().sum();
  return res;
}

vector_t GGTAbsRowSumInParallel(ThreadPool& threadPool, const OcpSize& ocpSize,
                                const std::vector<VectorFunctionLinearApproximation>& dynamics,
                                const std::vector<VectorFunctionLinearApproximation>* constraints,
                                const vector_array_t* scalingVectorsPtr) {
  const int N = ocpSize.numStages;
  if (N < 1) {
    throw std::runtime_error("[GGTAbsRowSumInParallel] The number of stages cannot be less than 1.");
  }
  if (scalingVectorsPtr != nullptr && scalingVectorsPtr->size() != N) {
    throw std::runtime_error("[GGTAbsRowSumInParallel] The size of scalingVectors doesn't match the number of stage.");
  }
  Eigen::setNbThreads(1);  // No multithreading within Eigen.

  matrix_array_t tempMatrixArray(N);
  vector_array_t absRowSumArray(N);

  std::atomic_int timeIndex{0};
  auto task = [&](int workerId) {
    int k;
    while ((k = timeIndex++) < N) {
      const auto nx_next = ocpSize.numStates[k + 1];
      const auto& B = dynamics[k].dfdu;
      tempMatrixArray[k] =
          (scalingVectorsPtr == nullptr ? matrix_t::Identity(nx_next, nx_next)
                                        : (*scalingVectorsPtr)[k].cwiseProduct((*scalingVectorsPtr)[k]).asDiagonal().toDenseMatrix());
      tempMatrixArray[k] += B * B.transpose();

      if (k != 0) {
        const auto& A = dynamics[k].dfdx;
        tempMatrixArray[k] += A * A.transpose();
      }

      absRowSumArray[k] = tempMatrixArray[k].cwiseAbs().rowwise().sum();

      if (k != 0) {
        const auto& A = dynamics[k].dfdx;
        absRowSumArray[k] += (A * (scalingVectorsPtr == nullptr ? matrix_t::Identity(A.cols(), A.cols())
                                                                : (*scalingVectorsPtr)[k - 1].asDiagonal().toDenseMatrix()))
                                 .cwiseAbs()
                                 .rowwise()
                                 .sum();
      }
      if (k != N - 1) {
        const auto& ANext = dynamics[k + 1].dfdx;
        absRowSumArray[k] += (ANext * (scalingVectorsPtr == nullptr ? matrix_t::Identity(ANext.cols(), ANext.cols())
                                                                    : (*scalingVectorsPtr)[k].asDiagonal().toDenseMatrix()))
                                 .transpose()
                                 .cwiseAbs()
                                 .rowwise()
                                 .sum();
      }
    }
  };
  threadPool.runParallel(std::move(task), threadPool.numThreads() + 1U);

  vector_t res = vector_t::Zero(GetNumDynamicsConstraints(ocpSize));
  int curRow = 0;
  for (const auto& v : absRowSumArray) {
    res.segment(curRow, v.size()) = v;
    curRow += v.size();
  }

  Eigen::setNbThreads(0);  // Restore default setup.

  return res;
}

}  // namespace slp
}  // namespace bolza
