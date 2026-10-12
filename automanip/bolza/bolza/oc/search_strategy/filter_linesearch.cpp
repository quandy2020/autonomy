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

#include "bolza/oc/search_strategy/filter_linesearch.hpp"

namespace bolza {

std::pair<bool, FilterLinesearch::StepType> FilterLinesearch::AcceptStep(const PerformanceIndex& baselinePerformance,
                                                                         const PerformanceIndex& stepPerformance,
                                                                         scalar_t armijoDescentMetric) const {
  const scalar_t baselineConstraintViolation = TotalConstraintViolation(baselinePerformance);
  const scalar_t stepConstraintViolation = TotalConstraintViolation(stepPerformance);

  // Step acceptance and record step type
  if (stepConstraintViolation > g_max) {
    // High constraint violation. Only accept decrease in constraints.
    const bool accepted = stepConstraintViolation < ((1.0 - gamma_c) * baselineConstraintViolation);
    return std::make_pair(accepted, StepType::CONSTRAINT);

  } else if (stepConstraintViolation < g_min && baselineConstraintViolation < g_min && armijoDescentMetric < 0.0) {
    // With low violation and having a descent direction, require the armijo condition.
    const bool accepted = stepPerformance.merit < (baselinePerformance.merit + armijoFactor * armijoDescentMetric);
    return std::make_pair(accepted, StepType::COST);

  } else {
    // Medium violation: either merit or constraints decrease (with small gamma_c mixing of old constraints)
    const bool accepted = stepPerformance.merit < (baselinePerformance.merit - gamma_c * baselineConstraintViolation) ||
                          stepConstraintViolation < ((1.0 - gamma_c) * baselineConstraintViolation);
    return std::make_pair(accepted, StepType::DUAL);
  }
}

std::string ToString(const FilterLinesearch::StepType& stepType) {
  using StepType = FilterLinesearch::StepType;
  switch (stepType) {
    case StepType::COST:
      return "Cost";
    case StepType::CONSTRAINT:
      return "Constraint";
    case StepType::DUAL:
      return "Dual";
    case StepType::ZERO:
      return "Zero";
    case StepType::UNKNOWN:
    default:
      return "Unknown";
  }
}

scalar_t ArmijoDescentMetric(const std::vector<ScalarFunctionQuadraticApproximation>& cost, const vector_array_t& deltaXSol,
                             const vector_array_t& deltaUSol) {
  // To determine if the solution is a descent direction for the cost: compute gradient(cost)' * [dx; du]
  scalar_t metric = 0.0;
  for (int i = 0; i < cost.size(); i++) {
    if (cost[i].dfdx.size() > 0) {
      metric += cost[i].dfdx.dot(deltaXSol[i]);
    }
    if (cost[i].dfdu.size() > 0) {
      metric += cost[i].dfdu.dot(deltaUSol[i]);
    }
  }
  return metric;
}

}  // namespace bolza
