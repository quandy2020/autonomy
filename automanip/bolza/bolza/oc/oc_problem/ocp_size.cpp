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

#include "bolza/oc/oc_problem/ocp_size.hpp"

namespace bolza {
bool operator==(const OcpSize& lhs, const OcpSize& rhs) noexcept {
  // use && instead of &= to enable short-circuit evaluation
  bool same = lhs.numStages == rhs.numStages;
  same = same && (lhs.numInputs == rhs.numInputs);
  same = same && (lhs.numStates == rhs.numStates);
  same = same && (lhs.numInputBoxConstraints == rhs.numInputBoxConstraints);
  same = same && (lhs.numStateBoxConstraints == rhs.numStateBoxConstraints);
  same = same && (lhs.numIneqConstraints == rhs.numIneqConstraints);
  same = same && (lhs.numInputBoxSlack == rhs.numInputBoxSlack);
  same = same && (lhs.numStateBoxSlack == rhs.numStateBoxSlack);
  same = same && (lhs.numIneqSlack == rhs.numIneqSlack);
  return same;
}

OcpSize ExtractSizesFromProblem(const std::vector<VectorFunctionLinearApproximation>& dynamics,
                                const std::vector<ScalarFunctionQuadraticApproximation>& cost,
                                const std::vector<VectorFunctionLinearApproximation>* constraints) {
  const int numStages = dynamics.size();

  OcpSize problemSize(dynamics.size());

  // State inputs
  for (int k = 0; k < numStages; k++) {
    problemSize.numStates[k] = dynamics[k].dfdx.cols();
    problemSize.numInputs[k] = dynamics[k].dfdu.cols();
  }
  problemSize.numStates[numStages] = dynamics[numStages - 1].dfdx.rows();
  problemSize.numInputs[numStages] = 0;

  // Constraints
  if (constraints != nullptr) {
    for (int k = 0; k < numStages + 1; k++) {
      problemSize.numIneqConstraints[k] = (*constraints)[k].f.size();
    }
  }

  return problemSize;
}

}  // namespace bolza