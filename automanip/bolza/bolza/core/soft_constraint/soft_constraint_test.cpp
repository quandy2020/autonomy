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

#include <gtest/gtest.h>

#include <bolza/core/penalties/penalties.hpp>
#include <bolza/core/soft_constraint/state_input_soft_constraint.hpp>
#include <bolza/core/soft_constraint/state_soft_constraint.hpp>

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
class TestStateConstraint final : public bolza::StateConstraint {
 public:
  TestStateConstraint(bolza::ConstraintOrder constraintOrder, size_t numConstraints)
      : StateConstraint(constraintOrder), numConstraints_(numConstraints) {}
  ~TestStateConstraint() override = default;
  TestStateConstraint* Clone() const override { return new TestStateConstraint(*this); }

  size_t GetNumConstraints(bolza::scalar_t time) const override { return numConstraints_; }
  bolza::vector_t GetValue(bolza::scalar_t time, const bolza::vector_t& state, const bolza::PreComputation&) const override {
    return bolza::vector_t::Zero(numConstraints_);
  }
  bolza::VectorFunctionLinearApproximation GetLinearApproximation(bolza::scalar_t time, const bolza::vector_t& state,
                                                                 const bolza::PreComputation&) const override {
    return bolza::VectorFunctionLinearApproximation::Zero(numConstraints_, state.size());
  }
  bolza::VectorFunctionQuadraticApproximation GetQuadraticApproximation(bolza::scalar_t time, const bolza::vector_t& state,
                                                                       const bolza::PreComputation&) const override {
    return bolza::VectorFunctionQuadraticApproximation::Zero(numConstraints_, state.size());
  }

 private:
  size_t numConstraints_;
};

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
class TestStateInputConstraint : public bolza::StateInputConstraint {
 public:
  TestStateInputConstraint(bolza::ConstraintOrder constraintOrder, size_t numConstraints)
      : bolza::StateInputConstraint(constraintOrder), numConstraints_(numConstraints) {}
  ~TestStateInputConstraint() override = default;
  TestStateInputConstraint* Clone() const override { return new TestStateInputConstraint(*this); }

  size_t GetNumConstraints(bolza::scalar_t time) const override { return numConstraints_; }
  bolza::vector_t GetValue(bolza::scalar_t time, const bolza::vector_t& state, const bolza::vector_t& input,
                          const bolza::PreComputation&) const override {
    return bolza::vector_t::Zero(numConstraints_);
  }
  bolza::VectorFunctionLinearApproximation GetLinearApproximation(bolza::scalar_t time, const bolza::vector_t& state,
                                                                 const bolza::vector_t& input, const bolza::PreComputation&) const override {
    return bolza::VectorFunctionLinearApproximation::Zero(numConstraints_, state.size(), input.size());
  }
  bolza::VectorFunctionQuadraticApproximation GetQuadraticApproximation(bolza::scalar_t time, const bolza::vector_t& state,
                                                                       const bolza::vector_t& input,
                                                                       const bolza::PreComputation&) const override {
    return bolza::VectorFunctionQuadraticApproximation::Zero(numConstraints_, state.size(), input.size());
  }

 private:
  size_t numConstraints_;
};

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <class Constraint, class SoftConstraint>
std::unique_ptr<SoftConstraint> softConstraintFactory(size_t numConstraints, bolza::ConstraintOrder constraintOrder,
                                                      bool useSimilarPenalty) {
  // constraint
  auto constraintPtr = std::make_unique<Constraint>(constraintOrder, numConstraints);

  if (useSimilarPenalty) {
    // penalty function
    auto penaltyFunctionPtr = std::make_unique<bolza::RelaxedBarrierPenalty>(bolza::RelaxedBarrierPenalty::Config{10.0, 1.0});
    return std::make_unique<SoftConstraint>(std::move(constraintPtr), std::move(penaltyFunctionPtr));

  } else {
    std::vector<std::unique_ptr<bolza::PenaltyBase>> penaltyFunctionPtrArry;
    for (size_t i = 0; i < numConstraints; i++) {
      penaltyFunctionPtrArry.emplace_back(new bolza::RelaxedBarrierPenalty({10.0, 1.0}));
    }  // end of i loop
    return std::make_unique<SoftConstraint>(std::move(constraintPtr), std::move(penaltyFunctionPtrArry));
  }
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
TEST(testSoftConstraint, clone) {
  constexpr size_t numConstraints = 10;

  auto stateQuadraticConstraintPtr =
      softConstraintFactory<TestStateConstraint, bolza::StateSoftConstraint>(numConstraints, bolza::ConstraintOrder::Quadratic, true);
  EXPECT_NO_THROW(std::unique_ptr<bolza::StateSoftConstraint>(stateQuadraticConstraintPtr->Clone()));

  auto stateLinearConstraintPtr =
      softConstraintFactory<TestStateInputConstraint, bolza::StateInputSoftConstraint>(numConstraints, bolza::ConstraintOrder::Linear, false);
  EXPECT_NO_THROW(std::unique_ptr<bolza::StateInputSoftConstraint>(stateLinearConstraintPtr->Clone()));
}

TEST(testSoftConstraint, softStateConstraint) {
  constexpr size_t numConstraints = 10;
  const bolza::vector_t state = bolza::vector_t::Zero(2);
  const bolza::vector_t input = bolza::vector_t::Zero(1);
  const bolza::TargetTrajectories targetTrajectories;
  const bolza::PreComputation preComp;

  auto stateLinearConstraintPtr =
      softConstraintFactory<TestStateConstraint, bolza::StateSoftConstraint>(numConstraints, bolza::ConstraintOrder::Linear, false);
  EXPECT_NO_THROW(stateLinearConstraintPtr->GetValue(0.0, state, targetTrajectories, preComp));
  EXPECT_NO_THROW(stateLinearConstraintPtr->GetQuadraticApproximation(0.0, state, targetTrajectories, preComp));

  auto stateQuadraticConstraintPtr =
      softConstraintFactory<TestStateConstraint, bolza::StateSoftConstraint>(numConstraints, bolza::ConstraintOrder::Quadratic, true);
  EXPECT_NO_THROW(stateQuadraticConstraintPtr->GetValue(0.0, state, targetTrajectories, preComp));
  EXPECT_NO_THROW(stateQuadraticConstraintPtr->GetQuadraticApproximation(0.0, state, targetTrajectories, preComp));
}

TEST(testSoftConstraint, softStateInputConstraint) {
  constexpr size_t numConstraints = 10;
  const bolza::vector_t state = bolza::vector_t::Zero(2);
  const bolza::vector_t input = bolza::vector_t::Zero(1);
  const bolza::TargetTrajectories targetTrajectories;
  const bolza::PreComputation preComp;

  auto stateInputLinearConstraintPtr =
      softConstraintFactory<TestStateInputConstraint, bolza::StateInputSoftConstraint>(numConstraints, bolza::ConstraintOrder::Linear, true);
  EXPECT_NO_THROW(stateInputLinearConstraintPtr->GetValue(0.0, state, input, targetTrajectories, preComp));
  EXPECT_NO_THROW(stateInputLinearConstraintPtr->GetQuadraticApproximation(0.0, state, input, targetTrajectories, preComp));

  auto stateInputQuadraticConstraintPtr = softConstraintFactory<TestStateInputConstraint, bolza::StateInputSoftConstraint>(
      numConstraints, bolza::ConstraintOrder::Quadratic, false);
  EXPECT_NO_THROW(stateInputQuadraticConstraintPtr->GetValue(0.0, state, input, targetTrajectories, preComp));
  EXPECT_NO_THROW(stateInputQuadraticConstraintPtr->GetQuadraticApproximation(0.0, state, input, targetTrajectories, preComp));
}

class ActivityTestStateConstraint : public bolza::StateConstraint {
 public:
  ActivityTestStateConstraint() : StateConstraint(bolza::ConstraintOrder::Quadratic) {}
  ~ActivityTestStateConstraint() override = default;
  ActivityTestStateConstraint(const ActivityTestStateConstraint& other) = default;
  ActivityTestStateConstraint* Clone() const override { return new ActivityTestStateConstraint(*this); }

  size_t GetNumConstraints(bolza::scalar_t time) const override { return 1; }
  bool IsActive(bolza::scalar_t) const override { return active_; }
  void SetActivity(bool active) { active_ = active; }
  bolza::vector_t GetValue(bolza::scalar_t time, const bolza::vector_t& state, const bolza::PreComputation&) const override {
    return bolza::vector_t::Zero(1);
  }

 private:
  bool active_ = true;
};

TEST(testSoftConstraint, checkActivityStateConstraint) {
  constexpr size_t numConstraints = 10;

  auto constraint = std::make_unique<ActivityTestStateConstraint>();
  auto penalty = std::make_unique<bolza::RelaxedBarrierPenalty>(bolza::RelaxedBarrierPenalty::Config{10.0, 1.0});
  auto softConstraint = std::make_unique<bolza::StateSoftConstraint>(std::move(constraint), std::move(penalty));

  softConstraint->get<ActivityTestStateConstraint>().setActivity(true);
  EXPECT_TRUE(std::unique_ptr<bolza::StateCost>(softConstraint->Clone())->IsActive(0.0));
  softConstraint->get<ActivityTestStateConstraint>().setActivity(false);
  EXPECT_FALSE(std::unique_ptr<bolza::StateCost>(softConstraint->Clone())->IsActive(0.0));
}

class ActivityTestStateInputConstraint : public bolza::StateInputConstraint {
 public:
  ActivityTestStateInputConstraint() : StateInputConstraint(bolza::ConstraintOrder::Quadratic) {}
  ~ActivityTestStateInputConstraint() override = default;
  ActivityTestStateInputConstraint(const ActivityTestStateInputConstraint& other) = default;
  ActivityTestStateInputConstraint* Clone() const override { return new ActivityTestStateInputConstraint(*this); }

  size_t GetNumConstraints(bolza::scalar_t time) const override { return 1; }
  bool IsActive(bolza::scalar_t) const override { return active_; }
  void SetActivity(bool active) { active_ = active; }
  bolza::vector_t GetValue(bolza::scalar_t time, const bolza::vector_t& state, const bolza::vector_t& input,
                          const bolza::PreComputation&) const override {
    return bolza::vector_t::Zero(1);
  }

 private:
  bool active_ = true;
};

TEST(testSoftConstraint, checkActivityStateInputConstraint) {
  constexpr size_t numConstraints = 10;

  auto constraint = std::make_unique<ActivityTestStateInputConstraint>();
  auto penalty = std::make_unique<bolza::RelaxedBarrierPenalty>(bolza::RelaxedBarrierPenalty::Config{10.0, 1.0});
  auto softConstraint = std::make_unique<bolza::StateInputSoftConstraint>(std::move(constraint), std::move(penalty));

  softConstraint->get<ActivityTestStateInputConstraint>().setActivity(true);
  EXPECT_TRUE(std::unique_ptr<bolza::StateInputCost>(softConstraint->Clone())->IsActive(0.0));
  softConstraint->get<ActivityTestStateInputConstraint>().setActivity(false);
  EXPECT_FALSE(std::unique_ptr<bolza::StateInputCost>(softConstraint->Clone())->IsActive(0.0));
}
