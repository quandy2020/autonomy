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

#include <numeric>

#include <gtest/gtest.h>

#include <bolza/core/constraint/state_constraint_collection.hpp>
#include <bolza/core/constraint/state_input_constraint_collection.hpp>
#include "testConstraints.h"

TEST(TestConstraintCollection, add) {
  bolza::StateInputConstraintCollection constraintCollection;

  // Add after construction
  auto constraintTerm = std::make_unique<TestEmptyConstraint>();
  ASSERT_NO_THROW({ constraintCollection.Add("Constraint1", std::move(constraintTerm)); });
}

TEST(TestConstraintCollection, numberOfConstraints) {
  bolza::StateInputConstraintCollection constraintCollection;

  // Initially we have zero constraints for all types
  EXPECT_EQ(constraintCollection.GetNumConstraints(0.0), 0);

  // Add Linear inequality constraint term, which has 2 constraints
  auto constraintTerm = std::make_unique<TestDummyConstraint>();
  const size_t addedConstraints = constraintTerm->GetNumConstraints(0.0);
  constraintCollection.Add("Constraint1", std::move(constraintTerm));

  // Check the right constraint size is incremented
  EXPECT_EQ(constraintCollection.GetNumConstraints(0.0), addedConstraints);
}

TEST(TestConstraintCollection, termsSize) {
  bolza::StateInputConstraintCollection constraintCollection;

  // Initially we have zero constraints for all types
  auto termsSize = constraintCollection.GetTermsSize(0.0);
  EXPECT_EQ(termsSize.size(), 0);
  EXPECT_EQ(std::accumulate(termsSize.begin(), termsSize.end(), 0), 0);

  // Add 2 Linear inequality constraint term, which has 2 constraints
  constraintCollection.Add("Constraint1", std::make_unique<TestDummyConstraint>());
  constraintCollection.Add("Constraint2", std::make_unique<TestDummyConstraint>());
  constraintCollection.Add("Constraint3", std::make_unique<TestDummyConstraint>());
  auto& constraint1 = constraintCollection.get<TestDummyConstraint>("Constraint1");
  const size_t constraint1Size = constraint1.GetNumConstraints(0.0);

  // Check the right constraint size
  termsSize = constraintCollection.GetTermsSize(0.0);
  EXPECT_EQ(termsSize.size(), 3);
  if (termsSize.size() == 3) {
    EXPECT_EQ(termsSize[0], constraint1Size);
    EXPECT_EQ(termsSize[1], constraint1Size);
    EXPECT_EQ(termsSize[2], constraint1Size);
  }

  // Deactivate constraint1
  constraint1.setActivity(false);
  termsSize = constraintCollection.GetTermsSize(0.0);
  EXPECT_EQ(termsSize.size(), 3);
  EXPECT_EQ(std::accumulate(termsSize.begin(), termsSize.end(), 0), 2 * constraint1Size);
}

TEST(TestConstraintCollection, activatingConstraints) {
  bolza::StateInputConstraintCollection constraintCollection;

  // Initially we have zero constraints for all types
  EXPECT_EQ(constraintCollection.GetNumConstraints(0.0), 0);

  // Add Linear inequality constraint term, which has 2 constraints
  auto constraintTerm = std::make_unique<TestDummyConstraint>();
  const size_t addedConstraints = constraintTerm->GetNumConstraints(0.0);
  constraintCollection.Add("Constraint1", std::move(constraintTerm));

  // Check the right constraint size is incremented
  EXPECT_EQ(constraintCollection.GetNumConstraints(0.0), addedConstraints);

  constraintCollection.get<TestDummyConstraint>("Constraint1").setActivity(false);

  // Check the right constraint size after deactivating the constraint
  EXPECT_EQ(constraintCollection.GetNumConstraints(0.0), 0);
}

TEST(TestConstraintCollection, clone) {
  bolza::StateInputConstraintCollection constraintCollection;
  auto constraintTerm = std::make_unique<TestDummyConstraint>();
  const size_t addedConstraints = constraintTerm->GetNumConstraints(0.0);
  constraintCollection.Add("Constraint1", std::move(constraintTerm));

  // move construct
  std::unique_ptr<bolza::StateInputConstraintCollection> newCollection(constraintCollection.Clone());
  EXPECT_EQ(newCollection->GetNumConstraints(0.0), addedConstraints);
}

TEST(TestConstraintCollection, getValue) {
  using collection_t = bolza::StateInputConstraintCollection;
  collection_t constraintCollection;

  // evaluation point
  double t = 0.0;
  bolza::vector_t x(3);
  bolza::vector_t u(2);
  u.setZero();
  x.setZero();

  // Zero constraints after creating
  auto constraintValues = constraintCollection.GetValue(t, x, u, bolza::PreComputation());
  EXPECT_EQ(constraintValues.size(), 0);

  // Add Linear inequality constraint term, which has 2 constraints, twice
  auto constraintTerm1 = std::make_unique<TestDummyConstraint>();
  auto constraintTerm2 = std::make_unique<TestDummyConstraint>();
  constraintCollection.Add("Constraint1", std::move(constraintTerm1));
  constraintCollection.Add("Constraint2", std::move(constraintTerm2));
  const bolza::vector_t expectedValue = (bolza::vector_t(2) << 1.0, 2.0).finished();
  constraintValues = constraintCollection.GetValue(t, x, u, bolza::PreComputation());

  ASSERT_EQ(constraintValues.size(), 2);
  EXPECT_TRUE(constraintValues[0].isApprox(expectedValue));
  EXPECT_TRUE(constraintValues[1].isApprox(expectedValue));
}

TEST(TestConstraintCollection, getLinearApproximation) {
  using collection_t = bolza::StateInputConstraintCollection;
  collection_t constraintCollection;

  // evaluation point
  double t = 0.0;
  bolza::vector_t x(3);
  bolza::vector_t u(2);
  u.setZero();
  x.setZero();

  // Add Linear inequality constraint term, which has 2 constraints, twice
  auto constraintTerm1 = std::make_unique<TestDummyConstraint>();
  auto constraintTerm2 = std::make_unique<TestDummyConstraint>();
  constraintCollection.Add("Constraint1", std::move(constraintTerm1));
  constraintCollection.Add("Constraint2", std::move(constraintTerm2));

  auto linearApproximation = constraintCollection.GetLinearApproximation(t, x, u, bolza::PreComputation());
  ASSERT_EQ(linearApproximation.f.size(), 4);
  EXPECT_EQ(linearApproximation.f(0), 1.0);
  EXPECT_EQ(linearApproximation.f(1), 2.0);
  EXPECT_EQ(linearApproximation.f(2), 1.0);
  EXPECT_EQ(linearApproximation.f(3), 2.0);
  EXPECT_EQ(linearApproximation.dfdx.row(0).sum(), 0);
  EXPECT_EQ(linearApproximation.dfdx.row(1).sum(), 3);
  EXPECT_EQ(linearApproximation.dfdx.row(2).sum(), 0);
  EXPECT_EQ(linearApproximation.dfdx.row(3).sum(), 3);
  EXPECT_EQ(linearApproximation.dfdu.row(0).sum(), 0);
  EXPECT_EQ(linearApproximation.dfdu.row(1).sum(), 2);
  EXPECT_EQ(linearApproximation.dfdu.row(2).sum(), 0);
  EXPECT_EQ(linearApproximation.dfdu.row(3).sum(), 2);
}

TEST(TestConstraintCollection, getQuadraticApproximation) {
  using collection_t = bolza::StateInputConstraintCollection;
  collection_t constraintCollection;

  // evaluation point
  double t = 0.0;
  bolza::vector_t x(3);
  bolza::vector_t u(2);
  u.setZero();
  x.setZero();

  // Add Linear inequality constraint term, which has 2 constraints, twice
  auto constraintTerm1 = std::make_unique<TestDummyConstraint>();
  auto constraintTerm2 = std::make_unique<TestDummyConstraint>();
  constraintCollection.Add("Constraint1", std::move(constraintTerm1));
  constraintCollection.Add("Constraint2", std::move(constraintTerm2));

  auto quadraticApproximation = constraintCollection.GetQuadraticApproximation(t, x, u, bolza::PreComputation());
  ASSERT_EQ(quadraticApproximation.f.size(), 4);
  EXPECT_EQ(quadraticApproximation.f(0), 1.0);
  EXPECT_EQ(quadraticApproximation.f(1), 2.0);
  EXPECT_EQ(quadraticApproximation.f(2), 1.0);
  EXPECT_EQ(quadraticApproximation.f(3), 2.0);
  EXPECT_EQ(quadraticApproximation.dfdx.row(0).sum(), 0);
  EXPECT_EQ(quadraticApproximation.dfdx.row(1).sum(), 3);
  EXPECT_EQ(quadraticApproximation.dfdx.row(2).sum(), 0);
  EXPECT_EQ(quadraticApproximation.dfdx.row(3).sum(), 3);
  EXPECT_EQ(quadraticApproximation.dfdu.row(0).sum(), 0);
  EXPECT_EQ(quadraticApproximation.dfdu.row(1).sum(), 2);
  EXPECT_EQ(quadraticApproximation.dfdu.row(2).sum(), 0);
  EXPECT_EQ(quadraticApproximation.dfdu.row(3).sum(), 2);
  EXPECT_EQ(quadraticApproximation.dfdxx[0].sum(), 0);
  EXPECT_EQ(quadraticApproximation.dfduu[0].sum(), 0);
  EXPECT_EQ(quadraticApproximation.dfdux[0].sum(), 0);
  EXPECT_EQ(quadraticApproximation.dfdxx[1].sum(), 3 * 3);
  EXPECT_EQ(quadraticApproximation.dfduu[1].sum(), 2 * 2);
  EXPECT_EQ(quadraticApproximation.dfdux[1].sum(), 2 * 3);
}
