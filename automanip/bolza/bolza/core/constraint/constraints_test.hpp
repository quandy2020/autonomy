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

#pragma once

#include <bolza/core/constraint/state_input_constraint.hpp>

/** Empty constraint with 0 entries */
class TestEmptyConstraint final : public bolza::StateInputConstraint {
 public:
  using LinearApproximation_t = bolza::VectorFunctionLinearApproximation;

  TestEmptyConstraint() : bolza::StateInputConstraint(bolza::ConstraintOrder::Linear) {}
  ~TestEmptyConstraint() override = default;
  TestEmptyConstraint* Clone() const override { return new TestEmptyConstraint(*this); }

  size_t GetNumConstraints(bolza::scalar_t time) const override { return 0; };

  bolza::vector_t GetValue(bolza::scalar_t time, const bolza::vector_t& state, const bolza::vector_t& input,
                          const bolza::PreComputation&) const override {
    return bolza::vector_t(0);
  }

  LinearApproximation_t GetLinearApproximation(bolza::scalar_t time, const bolza::vector_t& state, const bolza::vector_t& input,
                                               const bolza::PreComputation&) const override {
    LinearApproximation_t linearApproximation;
    linearApproximation.resize(0, state.rows(), input.rows());
    return linearApproximation;
  }
};

/** Dummy state-input constraint with 2 entries */
class TestDummyConstraint final : public bolza::StateInputConstraint {
 public:
  using LinearApproximation_t = bolza::VectorFunctionLinearApproximation;
  using QuadraticApproximation_t = bolza::VectorFunctionQuadraticApproximation;

  TestDummyConstraint() : bolza::StateInputConstraint(bolza::ConstraintOrder::Quadratic) {}
  ~TestDummyConstraint() override = default;
  TestDummyConstraint* Clone() const override { return new TestDummyConstraint(*this); }

  size_t GetNumConstraints(bolza::scalar_t time) const override { return 2; }

  bool IsActive(bolza::scalar_t) const override { return active_; }

  void SetActivity(bool active) { active_ = active; }

  bolza::vector_t GetValue(bolza::scalar_t time, const bolza::vector_t& state, const bolza::vector_t& input,
                          const bolza::PreComputation&) const override {
    bolza::vector_t ConstraintValues(2);
    ConstraintValues << 1, 2;
    return constraintValues;
  }

  LinearApproximation_t GetLinearApproximation(bolza::scalar_t time, const bolza::vector_t& state, const bolza::vector_t& input,
                                               const bolza::PreComputation&) const override {
    LinearApproximation_t linearApproximation;
    linearApproximation.setZero(2, state.rows(), input.rows());
    linearApproximation.f = GetValue(time, state, input, bolza::PreComputation());
    linearApproximation.dfdx.row(1).setOnes();
    linearApproximation.dfdu.row(1).setOnes();
    return linearApproximation;
  }

  QuadraticApproximation_t GetQuadraticApproximation(bolza::scalar_t time, const bolza::vector_t& state, const bolza::vector_t& input,
                                                     const bolza::PreComputation&) const override {
    QuadraticApproximation_t quadraticApproximation;
    quadraticApproximation.setZero(2, state.rows(), input.rows());
    quadraticApproximation.f = GetValue(time, state, input, bolza::PreComputation());
    quadraticApproximation.dfdx.row(1).setOnes();
    quadraticApproximation.dfdu.row(1).setOnes();
    quadraticApproximation.dfdxx[1].setOnes();
    quadraticApproximation.dfdux[1].setOnes();
    quadraticApproximation.dfduu[1].setOnes();
    return quadraticApproximation;
  }

 private:
  bool active_ = true;
};

/** Dummy state-only constraint with 2 entries */
class TestDummyStateConstraint final : public bolza::StateConstraint {
 public:
  using LinearApproximation_t = bolza::VectorFunctionLinearApproximation;
  using QuadraticApproximation_t = bolza::VectorFunctionQuadraticApproximation;

  TestDummyStateConstraint() : bolza::StateConstraint(bolza::ConstraintOrder::Quadratic) {}
  ~TestDummyStateConstraint() override = default;
  TestDummyStateConstraint* Clone() const override { return new TestDummyStateConstraint(*this); }

  size_t GetNumConstraints(bolza::scalar_t time) const override { return 2; }

  bolza::vector_t GetValue(bolza::scalar_t time, const bolza::vector_t& state, const bolza::PreComputation&) const override {
    bolza::vector_t ConstraintValues(2);
    ConstraintValues << 1, 2;
    return constraintValues;
  }

  LinearApproximation_t GetLinearApproximation(bolza::scalar_t time, const bolza::vector_t& state,
                                               const bolza::PreComputation&) const override {
    LinearApproximation_t linearApproximation;
    linearApproximation.setZero(2, state.rows());
    linearApproximation.f = GetValue(time, state, bolza::PreComputation());
    linearApproximation.dfdx.row(1).setOnes();
    return linearApproximation;
  }

  QuadraticApproximation_t GetQuadraticApproximation(bolza::scalar_t time, const bolza::vector_t& state,
                                                     const bolza::PreComputation&) const override {
    QuadraticApproximation_t quadraticApproximation;
    quadraticApproximation.setZero(2, state.rows());
    quadraticApproximation.f = GetValue(time, state, bolza::PreComputation());
    quadraticApproximation.dfdx.row(1).setOnes();
    quadraticApproximation.dfdxx[1].setOnes();
    return quadraticApproximation;
  }
};
