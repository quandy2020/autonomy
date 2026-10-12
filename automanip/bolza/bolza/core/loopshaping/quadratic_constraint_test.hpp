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

#include <bolza/core/constraint/state_constraint.hpp>
#include <bolza/core/constraint/state_input_constraint.hpp>

namespace bolza {

class TestQuadraticStateInputConstraint : public StateInputConstraint {
 public:
  TestQuadraticStateInputConstraint(matrix_t Q, matrix_t R, matrix_t P)
      : StateInputConstraint(ConstraintOrder::Quadratic), Q_(std::move(Q)), R_(std::move(R)), P_(std::move(P)) {}

  static std::unique_ptr<TestQuadraticStateInputConstraint> CreateRandom(size_t stateDim, size_t inputDim) {
    matrix_t Q, R, P;
    Q.setRandom(stateDim, stateDim);
    R.setRandom(inputDim, inputDim);
    P.setRandom(inputDim, stateDim);
    Q = (0.5 * Q.transpose() + 0.5 * Q).eval();
    R = (0.5 * R.transpose() + 0.5 * R).eval();
    return std::make_unique<TestQuadraticStateInputConstraint>(std::move(Q), std::move(R), std::move(P));
  }

  ~TestQuadraticStateInputConstraint() override = default;

  TestQuadraticStateInputConstraint* Clone() const override { return new TestQuadraticStateInputConstraint(*this); }

  size_t GetNumConstraints(scalar_t time) const final { return 1; }

  vector_t GetValue(scalar_t t, const vector_t& x, const vector_t& u, const PreComputation&) const final {
    vector_t c(1);
    c(0) = 0.5 * x.dot(Q_ * x) + 0.5 * u.dot(R_ * u) + u.dot(P_ * x);
    return c;
  }

  VectorFunctionLinearApproximation GetLinearApproximation(scalar_t t, const vector_t& x, const vector_t& u,
                                                           const PreComputation& preComp) const final {
    VectorFunctionLinearApproximation c;
    c.f = GetValue(t, x, u, preComp);
    c.dfdx = x.transpose() * Q_ + u.transpose() * P_;
    c.dfdu = u.transpose() * R_ + x.transpose() * P_.transpose();
    return c;
  }

  VectorFunctionQuadraticApproximation GetQuadraticApproximation(scalar_t t, const vector_t& x, const vector_t& u,
                                                                 const PreComputation& preComp) const final {
    VectorFunctionQuadraticApproximation c;
    c.f = GetValue(t, x, u, preComp);
    c.dfdx = x.transpose() * Q_ + u.transpose() * P_;
    c.dfdu = u.transpose() * R_ + x.transpose() * P_.transpose();
    c.dfdxx.push_back(Q_);
    c.dfduu.push_back(R_);
    c.dfdux.push_back(P_);
    return c;
  }

 public:
  matrix_t Q_;
  matrix_t R_;
  matrix_t P_;
};

class TestQuadraticStateConstraint : public StateConstraint {
 public:
  TestQuadraticStateConstraint(matrix_t Q) : StateConstraint(ConstraintOrder::Quadratic), Q_(std::move(Q)) {}

  static std::unique_ptr<TestQuadraticStateConstraint> CreateRandom(size_t stateDim) {
    matrix_t Q;
    Q.setRandom(stateDim, stateDim);
    Q = (0.5 * Q.transpose() + 0.5 * Q).eval();
    return std::make_unique<TestQuadraticStateConstraint>(std::move(Q));
  }

  ~TestQuadraticStateConstraint() override = default;

  TestQuadraticStateConstraint* Clone() const override { return new TestQuadraticStateConstraint(*this); }

  size_t GetNumConstraints(scalar_t time) const final { return 1; }

  vector_t GetValue(scalar_t t, const vector_t& x, const PreComputation&) const final {
    vector_t c(1);
    c(0) = 0.5 * x.dot(Q_ * x);
    return c;
  }

  VectorFunctionLinearApproximation GetLinearApproximation(scalar_t t, const vector_t& x, const PreComputation& preComp) const final {
    VectorFunctionLinearApproximation c;
    c.f = GetValue(t, x, preComp);
    c.dfdx = x.transpose() * Q_;
    return c;
  }

  VectorFunctionQuadraticApproximation GetQuadraticApproximation(scalar_t t, const vector_t& x, const PreComputation& preComp) const final {
    VectorFunctionQuadraticApproximation c;
    c.f = GetValue(t, x, preComp);
    c.dfdx = x.transpose() * Q_;
    c.dfdxx.push_back(Q_);
    return c;
  }

 public:
  matrix_t Q_;
};

}  // namespace bolza
