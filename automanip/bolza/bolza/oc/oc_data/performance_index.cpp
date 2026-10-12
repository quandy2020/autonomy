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

#include "bolza/oc/oc_data/performance_index.hpp"

#include <iomanip>

#include <bolza/core/misc/numerics.hpp>

namespace bolza {

PerformanceIndex& PerformanceIndex::operator+=(const PerformanceIndex& rhs) {
  this->merit += rhs.merit;
  this->cost += rhs.cost;
  this->dualFeasibilitiesSSE += rhs.dualFeasibilitiesSSE;
  this->dynamicsViolationSSE += rhs.dynamicsViolationSSE;
  this->equalityConstraintsSSE += rhs.equalityConstraintsSSE;
  this->inequalityConstraintsSSE += rhs.inequalityConstraintsSSE;
  this->equalityLagrangian += rhs.equalityLagrangian;
  this->inequalityLagrangian += rhs.inequalityLagrangian;
  return *this;
}

PerformanceIndex& PerformanceIndex::operator*=(const scalar_t c) {
  this->merit *= c;
  this->cost *= c;
  this->dualFeasibilitiesSSE *= c;
  this->dynamicsViolationSSE *= c;
  this->equalityConstraintsSSE *= c;
  this->inequalityConstraintsSSE *= c;
  this->equalityLagrangian *= c;
  this->inequalityLagrangian *= c;
  return *this;
}

bool PerformanceIndex::isApprox(const PerformanceIndex& other, const scalar_t prec) const {
  return numerics::almost_eq(this->merit, other.merit, prec) && numerics::almost_eq(this->cost, other.cost, prec) &&
         numerics::almost_eq(this->dualFeasibilitiesSSE, other.dualFeasibilitiesSSE, prec) &&
         numerics::almost_eq(this->dynamicsViolationSSE, other.dynamicsViolationSSE, prec) &&
         numerics::almost_eq(this->equalityConstraintsSSE, other.equalityConstraintsSSE, prec) &&
         numerics::almost_eq(this->inequalityConstraintsSSE, other.inequalityConstraintsSSE, prec) &&
         numerics::almost_eq(this->equalityLagrangian, other.equalityLagrangian, prec) &&
         numerics::almost_eq(this->inequalityLagrangian, other.inequalityLagrangian, prec);
}

void swap(PerformanceIndex& lhs, PerformanceIndex& rhs) {
  std::swap(lhs.merit, rhs.merit);
  std::swap(lhs.cost, rhs.cost);
  std::swap(lhs.dualFeasibilitiesSSE, rhs.dualFeasibilitiesSSE);
  std::swap(lhs.dynamicsViolationSSE, rhs.dynamicsViolationSSE);
  std::swap(lhs.equalityConstraintsSSE, rhs.equalityConstraintsSSE);
  std::swap(lhs.inequalityConstraintsSSE, rhs.inequalityConstraintsSSE);
  std::swap(lhs.equalityLagrangian, rhs.equalityLagrangian);
  std::swap(lhs.inequalityLagrangian, rhs.inequalityLagrangian);
}

PerformanceIndex ToPerformanceIndex(const Metrics& m) {
  PerformanceIndex performanceIndex;
  performanceIndex.merit = 0.0;  // left for the solver to fill
  performanceIndex.cost = m.cost;
  performanceIndex.dualFeasibilitiesSSE = 0.0;  // left for the solver to fill
  performanceIndex.dynamicsViolationSSE = GetEqConstraintsSSE(m.dynamicsViolation);
  performanceIndex.equalityConstraintsSSE = GetEqConstraintsSSE(m.stateEqConstraint) + GetEqConstraintsSSE(m.stateInputEqConstraint);
  performanceIndex.inequalityConstraintsSSE =
      GetIneqConstraintsSSE(m.stateIneqConstraint) + GetIneqConstraintsSSE(m.stateInputIneqConstraint);
  performanceIndex.equalityLagrangian = SumPenalties(m.stateEqLagrangian) + SumPenalties(m.stateInputEqLagrangian);
  performanceIndex.inequalityLagrangian = SumPenalties(m.stateIneqLagrangian) + SumPenalties(m.stateInputIneqLagrangian);
  return performanceIndex;
}

PerformanceIndex ToPerformanceIndex(const Metrics& m, const scalar_t dt) {
  auto performanceIndex = ToPerformanceIndex(m);
  //  performanceIndex.cost *= dt  no need since it is already considered in multiple_shooting::ComputeIntermediateMetrics()
  performanceIndex.dualFeasibilitiesSSE *= dt;
  performanceIndex.dynamicsViolationSSE *= dt;
  performanceIndex.equalityConstraintsSSE *= dt;
  performanceIndex.inequalityConstraintsSSE *= dt;
  return performanceIndex;
}

std::ostream& operator<<(std::ostream& stream, const PerformanceIndex& performanceIndex) {
  const size_t tabSpace = 12;
  const auto indentation = stream.width();
  stream << std::left;  // fill from left

  stream << std::setw(indentation) << "";
  stream << "Rollout Merit:              " << std::setw(tabSpace) << performanceIndex.merit;
  stream << "Rollout Cost:               " << std::setw(tabSpace) << performanceIndex.cost << '\n';

  stream << std::setw(indentation) << "";
  stream << "Dual feasibilities SSE:     " << std::setw(tabSpace) << performanceIndex.dualFeasibilitiesSSE;
  stream << "Dynamics violation SSE:     " << std::setw(tabSpace) << performanceIndex.dynamicsViolationSSE << '\n';

  stream << std::setw(indentation) << "";
  stream << "Equality constraints SSE:   " << std::setw(tabSpace) << performanceIndex.equalityConstraintsSSE;
  stream << "Inequality constraints SSE: " << std::setw(tabSpace) << performanceIndex.inequalityConstraintsSSE << '\n';

  stream << std::setw(indentation) << "";
  stream << "Equality Lagrangian:        " << std::setw(tabSpace) << performanceIndex.equalityLagrangian;
  stream << "Inequality Lagrangian:      " << std::setw(tabSpace) << performanceIndex.inequalityLagrangian;

  return stream;
}

}  // namespace bolza
