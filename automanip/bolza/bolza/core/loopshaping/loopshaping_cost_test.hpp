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

#include "bolza/core/cost/quadratic_state_cost.hpp"
#include "bolza/core/cost/quadratic_state_input_cost.hpp"
#include "bolza/core/loopshaping/cost/loopshaping_cost.hpp"

#include "testLoopshapingConfigurations.h"

namespace bolza {

class TestFixtureLoopShapingCost : LoopshapingTestConfiguration {
 public:
  TestFixtureLoopShapingCost(const std::string& configName) : LoopshapingTestConfiguration(configName) {
    // Create system costs
    matrix_t Q, Q_final, R, P;
    Q.setRandom(systemStateDim_, systemStateDim_);
    Q_final.setRandom(systemStateDim_, systemStateDim_);
    R.setRandom(inputDim_, inputDim_);
    P.setRandom(inputDim_, systemStateDim_);

    // Make symmetric
    Q_final = (0.5 * Q_final.transpose() + 0.5 * Q_final).eval();
    Q = (0.5 * Q.transpose() + 0.5 * Q).eval();
    R = (0.5 * R.transpose() + 0.5 * R).eval();
    systemCost.Reset(new QuadraticStateInputCost(Q, R, P));
    systemStateCost.Reset(new QuadraticStateCost(Q_final));

    targetTrajectories_ = TargetTrajectories({0.0}, {x_sys_}, {u_sys_});

    StateInputCostCollection systemCostCollection;
    StateCostCollection systemStateCostCollection;
    systemCostCollection.Add("", std::unique_ptr<StateInputCost>(systemCost->Clone()));
    systemStateCostCollection.Add("", std::unique_ptr<StateCost>(systemStateCost->Clone()));

    // Create Loopshaping cost collection wrappers
    loopshapingCost = LoopshapingCost::Create(systemCostCollection, loopshapingDefinition_);
    loopshapingStateCost = LoopshapingCost::Create(systemStateCostCollection, loopshapingDefinition_);
  }

  void TestStateInputCostApproximation() const {
    // Extract Quadratic approximation
    preComp_->Request(Request::Cost + Request::Approximation, t, x_, u_);
    const auto L = loopshapingCost->GetQuadraticApproximation(t, x_, u_, targetTrajectories_, *preComp_);

    // Reevaluate at disturbed state
    preComp_->Request(Request::Cost, t, x_ + x_disturbance_, u_ + u_disturbance_);
    const scalar_t L_disturbance = loopshapingCost->GetValue(t, x_ + x_disturbance_, u_ + u_disturbance_, targetTrajectories_, *preComp_);

    // Evaluate approximation
    const scalar_t L_quad_approximation = L.f + L.dfdx.dot(x_disturbance_) + L.dfdu.dot(u_disturbance_) +
                                          0.5 * x_disturbance_.dot(L.dfdxx * x_disturbance_) +
                                          0.5 * u_disturbance_.dot(L.dfduu * u_disturbance_) + u_disturbance_.dot(L.dfdux * x_disturbance_);

    // Difference between new evaluation and approximation should be less than tol
    EXPECT_NEAR(L_disturbance, L_quad_approximation, tol);
  }

  void TestStateCostApproximation() const {
    preComp_->RequestFinal(Request::Cost + Request::Approximation, t, x_);

    // Extract Quadratic approximation
    const auto L = loopshapingStateCost->GetQuadraticApproximation(t, x_, targetTrajectories_, *preComp_);

    // Reevaluate at disturbed state
    preComp_->RequestFinal(Request::Cost, t, x_ + x_disturbance_);
    const scalar_t L_disturbance = loopshapingStateCost->GetValue(t, x_ + x_disturbance_, targetTrajectories_, *preComp_);

    // Evaluate approximation
    const scalar_t L_quad_approximation = L.f + L.dfdx.dot(x_disturbance_) + 0.5 * x_disturbance_.dot(L.dfdxx * x_disturbance_);

    // Difference between new evaluation and approximation should be less than tol
    EXPECT_NEAR(L_disturbance, L_quad_approximation, tol);
  }

 private:
  std::unique_ptr<StateInputCost> systemCost;
  std::unique_ptr<StateCost> systemStateCost;
  std::unique_ptr<StateInputCostCollection> loopshapingCost;
  std::unique_ptr<StateCostCollection> loopshapingStateCost;
  TargetTrajectories targetTrajectories_;
};

};  // namespace bolza
