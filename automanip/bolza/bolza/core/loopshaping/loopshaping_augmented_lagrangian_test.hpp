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

#include <gtest/gtest.h>
#include <Eigen/Dense>

#include "bolza/core/augmented_lagrangian/augmented_lagrangian.hpp"
#include "bolza/core/loopshaping/loopshaping_definition.hpp"
#include "bolza/core/loopshaping/loopshaping_pre_computation.hpp"
#include "bolza/core/loopshaping/loopshaping_property_tree.hpp"
#include "bolza/core/loopshaping/augmented_lagrangian/loopshaping_augmented_lagrangian.hpp"
#include "bolza/core/penalties/penalties.hpp"

#include "testLoopshapingConfigurations.h"
#include "testQuadraticConstraint.h"

namespace bolza {

class TestFixtureLoopShapingAugmentedLagrangian : LoopshapingTestConfiguration {
 public:
  TestFixtureLoopShapingAugmentedLagrangian(const std::string& configName) : LoopshapingTestConfiguration(configName) {
    // Make system Constraint
    auto systemConstraint = TestQuadraticStateInputConstraint::CreateRandom(systemStateDim_, inputDim_);
    auto systemStateConstraint = TestQuadraticStateConstraint::CreateRandom(systemStateDim_);

    // Make Lagrangian: we use zero scale factor in order to make the quadratic approximation exact
    systemAugmentedLagrangian = Create(std::move(systemConstraint), augmented::QuadraticPenalty::Create({0.0, 1.0}));
    systemStateAugmentedLagrangian = Create(std::move(systemStateConstraint), augmented::QuadraticPenalty::Create({0.0, 1.0}));

    // Create temporary collection and add a constraint copy
    StateInputAugmentedLagrangianCollection systemAugmentedLagrangianCollection;
    StateAugmentedLagrangianCollection systemStateAugmentedLagrangianCollection;

    systemAugmentedLagrangianCollection.Add("", std::unique_ptr<StateInputAugmentedLagrangian>(systemAugmentedLagrangian->Clone()));
    systemStateAugmentedLagrangianCollection.Add("", std::unique_ptr<StateAugmentedLagrangian>(systemStateAugmentedLagrangian->Clone()));

    // Create Loopshaping constraint
    loopshapingAugmentedLagrangian = LoopshapingAugmentedLagrangian::Create(systemAugmentedLagrangianCollection, loopshapingDefinition_);
    loopshapingStateAugmentedLagrangian =
        LoopshapingAugmentedLagrangian::Create(systemStateAugmentedLagrangianCollection, loopshapingDefinition_);
  };

  void TestStateInputEvaluation() const {
    // Make random terms Multiplier
    const auto numConstraints = systemAugmentedLagrangian->GetNumConstraints(t);
    const auto termsMultiplier = ToMultipliers({numConstraints}, vector_t::Random(numConstraints + 1));

    // Evaluate system
    const auto metrics_system = systemAugmentedLagrangian->GetValue(t, x_sys_, u_sys_, termsMultiplier[0], PreComputation());

    // Evaluate loopshaping system
    preComp_->Request(Request::Constraint, t, x_, u_);
    const auto metrics = loopshapingAugmentedLagrangian->GetValue(t, x_, u_, termsMultiplier, *preComp_)[0];

    // The constraint and penalty should stay the same
    EXPECT_NEAR(metrics_system.penalty, metrics.penalty, tol);
    EXPECT_TRUE(metrics_system.constraint.isApprox(metrics.constraint, tol));
  }

  void TestStateInputApproximation() const {
    // Make random terms Multiplier
    const auto numConstraints = systemAugmentedLagrangian->GetNumConstraints(t);
    const auto termsMultiplier = ToMultipliers({numConstraints}, vector_t::Random(numConstraints + 1));

    // Extract Quadratic approximation
    preComp_->Request(Request::Constraint + Request::Approximation, t, x_, u_);
    const auto L = loopshapingAugmentedLagrangian->GetQuadraticApproximation(t, x_, u_, termsMultiplier, *preComp_);

    // Reevaluate at disturbed state
    preComp_->Request(Request::Constraint, t, x_ + x_disturbance_, u_ + u_disturbance_);
    const scalar_t L_disturbance =
        SumPenalties(loopshapingAugmentedLagrangian->GetValue(t, x_ + x_disturbance_, u_ + u_disturbance_, termsMultiplier, *preComp_));

    // Evaluate approximation
    const scalar_t L_quad_approximation = L.f + L.dfdx.dot(x_disturbance_) + L.dfdu.dot(u_disturbance_) +
                                          0.5 * x_disturbance_.dot(L.dfdxx * x_disturbance_) +
                                          0.5 * u_disturbance_.dot(L.dfduu * u_disturbance_) + u_disturbance_.dot(L.dfdux * x_disturbance_);

    // Difference between new evaluation and approximation should be less than tol
    EXPECT_NEAR(L_disturbance, L_quad_approximation, tol);
  }

  void TestStateEvaluation() const {
    // Make random terms Multiplier
    const auto numConstraints = systemAugmentedLagrangian->GetNumConstraints(t);
    const auto termsMultiplier = ToMultipliers({numConstraints}, vector_t::Random(numConstraints + 1));

    // Evaluate system
    const auto metrics_system = systemStateAugmentedLagrangian->GetValue(t, x_sys_, termsMultiplier[0], PreComputation());

    // Evaluate loopshaping system
    preComp_->RequestFinal(Request::Constraint, t, x_);
    const auto metrics = loopshapingStateAugmentedLagrangian->GetValue(t, x_, termsMultiplier, *preComp_)[0];

    // The constraint and penalty should stay the same
    EXPECT_NEAR(metrics_system.penalty, metrics.penalty, tol);
    EXPECT_TRUE(metrics_system.constraint.isApprox(metrics.constraint, tol));
  }

  void TestStateApproximation() const {
    // Make random terms Multiplier
    const auto numConstraints = systemAugmentedLagrangian->GetNumConstraints(t);
    const auto termsMultiplier = ToMultipliers({numConstraints}, vector_t::Random(numConstraints + 1));

    // Extract Quadratic approximation
    preComp_->RequestFinal(Request::Constraint + Request::Approximation, t, x_);
    const auto L = loopshapingStateAugmentedLagrangian->GetQuadraticApproximation(t, x_, termsMultiplier, *preComp_);

    // Reevaluate at disturbed state
    preComp_->RequestFinal(Request::Constraint, t, x_ + x_disturbance_);
    const scalar_t L_disturbance =
        SumPenalties(loopshapingStateAugmentedLagrangian->GetValue(t, x_ + x_disturbance_, termsMultiplier, *preComp_));

    // Evaluate approximation
    const scalar_t L_quad_approximation = L.f + L.dfdx.dot(x_disturbance_) + 0.5 * x_disturbance_.dot(L.dfdxx * x_disturbance_);

    // Difference between new evaluation and approximation should be less than tol
    EXPECT_NEAR(L_disturbance, L_quad_approximation, tol);
  }

 private:
  std::unique_ptr<StateInputAugmentedLagrangian> systemAugmentedLagrangian;
  std::unique_ptr<StateAugmentedLagrangian> systemStateAugmentedLagrangian;

  std::unique_ptr<StateInputAugmentedLagrangianCollection> loopshapingAugmentedLagrangian;
  std::unique_ptr<StateAugmentedLagrangianCollection> loopshapingStateAugmentedLagrangian;
};

}  // namespace bolza
