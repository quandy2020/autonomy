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

#include "bolza/ipm/ipm_helpers.hpp"

#include <cassert>

namespace bolza {
namespace ipm {

void CondenseIneqConstraints(scalar_t barrierParam, const vector_t& slack, const vector_t& dual,
                             const VectorFunctionLinearApproximation& ineqConstraint, ScalarFunctionQuadraticApproximation& lagrangian) {
  assert(barrierParam > 0.0);
  const size_t nc = ineqConstraint.f.size();
  const size_t nu = ineqConstraint.dfdu.cols();

  if (nc == 0) {
    return;
  }

  // dual feasibilities
  lagrangian.dfdx.noalias() -= ineqConstraint.dfdx.transpose() * dual;
  if (ineqConstraint.dfdu.cols() > 0) {
    lagrangian.dfdu.noalias() -= ineqConstraint.dfdu.transpose() * dual;
  }

  // coefficients for condensing
  const vector_t condensingLinearCoeff = (dual.array() * ineqConstraint.f.array() - barrierParam) / slack.array();
  const vector_t condensingQuadraticCoeff = dual.cwiseQuotient(slack);

  // condensing
  lagrangian.dfdx.noalias() += ineqConstraint.dfdx.transpose() * condensingLinearCoeff;
  const matrix_t condensingQuadraticCoeff_dfdx = condensingQuadraticCoeff.asDiagonal() * ineqConstraint.dfdx;
  lagrangian.dfdxx.noalias() += ineqConstraint.dfdx.transpose() * condensingQuadraticCoeff_dfdx;

  if (nu > 0) {
    lagrangian.dfdu.noalias() += ineqConstraint.dfdu.transpose() * condensingLinearCoeff;
    const matrix_t condensingQuadraticCoeff_dfdu = condensingQuadraticCoeff.asDiagonal() * ineqConstraint.dfdu;
    lagrangian.dfduu.noalias() += ineqConstraint.dfdu.transpose() * condensingQuadraticCoeff_dfdu;
    lagrangian.dfdux.noalias() += ineqConstraint.dfdu.transpose() * condensingQuadraticCoeff_dfdx;
  }
}

vector_t RetrieveSlackDirection(const VectorFunctionLinearApproximation& stateInputIneqConstraints, const vector_t& dx, const vector_t& du,
                                scalar_t barrierParam, const vector_t& slackStateInputIneq) {
  assert(barrierParam > 0.0);
  if (stateInputIneqConstraints.f.size() == 0) {
    return vector_t();
  }

  vector_t slackDirection = stateInputIneqConstraints.f - slackStateInputIneq;
  slackDirection.noalias() += stateInputIneqConstraints.dfdx * dx;
  slackDirection.noalias() += stateInputIneqConstraints.dfdu * du;
  return slackDirection;
}

vector_t RetrieveSlackDirection(const VectorFunctionLinearApproximation& stateIneqConstraints, const vector_t& dx, scalar_t barrierParam,
                                const vector_t& slackStateIneq) {
  assert(barrierParam > 0.0);
  if (stateIneqConstraints.f.size() == 0) {
    return vector_t();
  }

  vector_t slackDirection = stateIneqConstraints.f - slackStateIneq;
  slackDirection.noalias() += stateIneqConstraints.dfdx * dx;
  return slackDirection;
}

vector_t RetrieveDualDirection(scalar_t barrierParam, const vector_t& slack, const vector_t& dual, const vector_t& slackDirection) {
  assert(barrierParam > 0.0);
  vector_t dualDirection = dual.cwiseProduct(slack + slackDirection);
  dualDirection.array() -= barrierParam;
  dualDirection.array() /= -slack.array();
  return dualDirection;
}

scalar_t FractionToBoundaryStepSize(const vector_t& v, const vector_t& dv, scalar_t marginRate) {
  assert(marginRate > 0.0);
  assert(marginRate <= 1.0);

  if (v.size() == 0) {
    return 1.0;
  }

  const vector_t invFractionToBoundary = (-1.0 / marginRate) * dv.cwiseQuotient(v);
  const auto alpha = invFractionToBoundary.maxCoeff();
  return alpha > 0.0 ? std::min(1.0 / alpha, 1.0) : 1.0;
}

namespace {
MultiplierCollection toMultiplierCollection(const multiple_shooting::ConstraintsSize constraintsSize, const vector_t& stateIneq) {
  MultiplierCollection multiplierCollection;
  size_t head = 0;
  for (const size_t size : constraintsSize.stateIneq) {
    multiplierCollection.stateIneq.emplace_back(0.0, stateIneq.segment(head, size));
    head += size;
  }
  return multiplierCollection;
}

MultiplierCollection toMultiplierCollection(const multiple_shooting::ConstraintsSize constraintsSize, const vector_t& stateIneq,
                                            const vector_t& stateInputIneq) {
  MultiplierCollection multiplierCollection = toMultiplierCollection(constraintsSize, stateIneq);
  size_t head = 0;
  for (const size_t size : constraintsSize.stateInputIneq) {
    multiplierCollection.stateInputIneq.emplace_back(0.0, stateInputIneq.segment(head, size));
    head += size;
  }
  return multiplierCollection;
}

vector_t extractLagrangian(const std::vector<Multiplier>& termsMultiplier) {
  size_t n = 0;
  std::for_each(termsMultiplier.begin(), termsMultiplier.end(), [&](const Multiplier& m) { n += m.lagrangian.size(); });

  vector_t vec(n);
  size_t head = 0;
  for (const auto& m : termsMultiplier) {
    vec.segment(head, m.lagrangian.size()) = m.lagrangian;
    head += m.lagrangian.size();
  }  // end of i loop

  return vec;
}
}  // namespace

DualSolution ToDualSolution(const std::vector<AnnotatedTime>& time, const std::vector<multiple_shooting::ConstraintsSize>& constraintsSize,
                            const vector_array_t& stateIneq, const vector_array_t& stateInputIneq) {
  // Problem horizon
  const int N = static_cast<int>(time.size()) - 1;

  DualSolution dualSolution;
  dualSolution.timeTrajectory = ToInterpolationTime(time);
  dualSolution.postEventIndices = ToPostEventIndices(time);

  dualSolution.preJumps.reserve(dualSolution.postEventIndices.size());
  dualSolution.intermediates.reserve(time.size());

  for (int i = 0; i < N; ++i) {
    if (time[i].event == AnnotatedTime::Event::PreEvent) {
      dualSolution.preJumps.emplace_back(toMultiplierCollection(constraintsSize[i], stateIneq[i]));
      dualSolution.intermediates.push_back(dualSolution.intermediates.back());  // no event at the initial node
    } else {
      dualSolution.intermediates.emplace_back(toMultiplierCollection(constraintsSize[i], stateIneq[i], stateInputIneq[i]));
    }
  }
  dualSolution.final = toMultiplierCollection(constraintsSize[N], stateIneq[N]);
  dualSolution.intermediates.push_back(dualSolution.intermediates.back());

  return dualSolution;
}

std::pair<vector_t, vector_t> FromMultiplierCollection(const MultiplierCollection& multiplierCollection) {
  return std::make_pair(extractLagrangian(multiplierCollection.stateIneq), extractLagrangian(multiplierCollection.stateInputIneq));
}

}  // namespace ipm
}  // namespace bolza
