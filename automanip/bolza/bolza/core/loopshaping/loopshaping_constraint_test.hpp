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

#include "bolza/core/loopshaping/loopshaping_definition.hpp"
#include "bolza/core/loopshaping/loopshaping_pre_computation.hpp"
#include "bolza/core/loopshaping/loopshaping_property_tree.hpp"
#include "bolza/core/loopshaping/constraint/loopshaping_constraint.hpp"
#include "bolza/core/model_data/metrics.hpp"

#include "testLoopshapingConfigurations.h"
#include "testQuadraticConstraint.h"

namespace bolza {

class TestFixtureLoopShapingConstraint : LoopshapingTestConfiguration {
 public:
  TestFixtureLoopShapingConstraint(const std::string& configName) : LoopshapingTestConfiguration(configName) {
    // Make system Constraint
    systemConstraint = TestQuadraticStateInputConstraint::CreateRandom(systemStateDim_, inputDim_);
    systemStateConstraint = TestQuadraticStateConstraint::CreateRandom(systemStateDim_);

    // Create temporary collection and add a constraint copy
    StateInputConstraintCollection systemConstraintCollection;
    StateConstraintCollection systemStateConstraintCollection;
    systemConstraintCollection.Add("", std::unique_ptr<StateInputConstraint>(systemConstraint->Clone()));
    systemStateConstraintCollection.Add("", std::unique_ptr<StateConstraint>(systemStateConstraint->Clone()));

    // Create Loopshaping constraint
    loopshapingConstraint = LoopshapingConstraint::Create(systemConstraintCollection, loopshapingDefinition_);
    loopshapingStateConstraint = LoopshapingConstraint::Create(systemStateConstraintCollection, loopshapingDefinition_);
  };

  void TestStateInputConstraintEvaluation() const {
    // Evaluate system
    const auto g_system = systemConstraint->GetValue(t, x_sys_, u_sys_, PreComputation());

    // Evaluate loopshaping system
    preComp_->Request(Request::Constraint, t, x_, u_);
    const auto g = ToVector(loopshapingConstraint->GetValue(t, x_, u_, *preComp_));

    // The constraint should stay the same
    EXPECT_TRUE(g_system.size() == g.size());
    EXPECT_TRUE(g_system.isApprox(g, tol));
  }

  void TestStateInputConstraintLinearApproximation() const {
    // Extract approximation
    preComp_->Request(Request::Constraint + Request::Approximation, t, x_, u_);
    const auto g_linear = loopshapingConstraint->GetLinearApproximation(t, x_, u_, *preComp_);
    const auto g_quadratic = loopshapingConstraint->GetQuadraticApproximation(t, x_, u_, *preComp_);

    EXPECT_TRUE(g_linear.f.isApprox(g_quadratic.f));
    EXPECT_TRUE(g_linear.dfdx.isApprox(g_quadratic.dfdx));
    EXPECT_TRUE(g_linear.dfdu.isApprox(g_quadratic.dfdu));
  }

  void TestStateInputConstraintQuadraticApproximation() const {
    // Extract approximation
    preComp_->Request(Request::Constraint + Request::Approximation, t, x_, u_);
    const auto h = loopshapingConstraint->GetQuadraticApproximation(t, x_, u_, *preComp_);

    // Reevaluate at disturbed state
    preComp_->Request(Request::Constraint, t, x_ + x_disturbance_, u_ + u_disturbance_);
    const auto h_disturbance = ToVector(loopshapingConstraint->GetValue(t, x_ + x_disturbance_, u_ + u_disturbance_, *preComp_));

    // Evaluate approximation
    for (size_t i = 0; i < h.f.rows(); i++) {
      scalar_t h_approximation = h.f(i) + h.dfdx.row(i) * x_disturbance_ + h.dfdu.row(i) * u_disturbance_ +
                                 0.5 * x_disturbance_.transpose() * h.dfdxx[i] * x_disturbance_ +
                                 0.5 * u_disturbance_.transpose() * h.dfduu[i] * u_disturbance_ +
                                 u_disturbance_.transpose() * h.dfdux[i] * x_disturbance_;
      EXPECT_NEAR(h_disturbance[i], h_approximation, tol);
    }
  }

  void TestStateOnlyConstraintEvaluation() const {
    // Evaluate system
    const auto g_system = systemStateConstraint->GetValue(t, x_sys_, PreComputation());

    // Evaluate loopshaping system
    preComp_->RequestFinal(Request::Constraint, t, x_);
    const auto g = ToVector(loopshapingStateConstraint->GetValue(t, x_, *preComp_));

    // System part of the constraints should stay the same
    EXPECT_TRUE(g_system.isApprox(g, tol));
  }

  void TestStateOnlyConstraintLinearApproximation() const {
    // Extract approximation
    preComp_->RequestFinal(Request::Constraint + Request::Approximation, t, x_);
    const auto g_linear = loopshapingStateConstraint->GetLinearApproximation(t, x_, *preComp_);
    const auto g_quadratic = loopshapingStateConstraint->GetQuadraticApproximation(t, x_, *preComp_);

    EXPECT_TRUE(g_linear.f.isApprox(g_quadratic.f));
    EXPECT_TRUE(g_linear.dfdx.isApprox(g_quadratic.dfdx));
  }

  void TestStateOnlyConstraintQuadraticApproximation() const {
    // Extract approximation
    preComp_->RequestFinal(Request::Constraint + Request::Approximation, t, x_);
    const auto h = loopshapingStateConstraint->GetQuadraticApproximation(t, x_, *preComp_);

    // Reevaluate at disturbed state
    preComp_->RequestFinal(Request::Constraint, t, x_ + x_disturbance_);
    const auto h_disturbance = ToVector(loopshapingStateConstraint->GetValue(t, x_ + x_disturbance_, *preComp_));

    // Evaluate approximation
    for (size_t i = 0; i < h.f.rows(); i++) {
      const auto h_approximation = h.f(i) + h.dfdx.row(i) * x_disturbance_ + 0.5 * x_disturbance_.transpose() * h.dfdxx[i] * x_disturbance_;
      EXPECT_NEAR(h_disturbance[i], h_approximation, tol);
    }
  }

 private:
  std::unique_ptr<TestQuadraticStateInputConstraint> systemConstraint;
  std::unique_ptr<TestQuadraticStateConstraint> systemStateConstraint;
  std::unique_ptr<StateInputConstraintCollection> loopshapingConstraint;
  std::unique_ptr<StateConstraintCollection> loopshapingStateConstraint;
};

}  // namespace bolza
