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

#include "testLoopshapingConfigurations.h"

#include "bolza/core/dynamics/linear_system_dynamics.hpp"
#include "bolza/core/loopshaping/dynamics/loopshaping_dynamics.hpp"

namespace bolza {

class TestFixtureLoopShapingDynamics : LoopshapingTestConfiguration {
 public:
  TestFixtureLoopShapingDynamics(const std::string& configName) : LoopshapingTestConfiguration(configName) {
    // Create system dynamics
    matrix_t A, B, G;
    A.setRandom(systemStateDim_, systemStateDim_);
    B.setRandom(systemStateDim_, inputDim_);
    G.setRandom(systemStateDim_, systemStateDim_);
    testSystem.reset(new LinearSystemDynamics(A, B, G));

    // Create Loopshaping Dynamics
    testLoopshapingDynamics = LoopshapingDynamics::Create(*testSystem, loopshapingDefinition_);
  };

  void EvaluateDynamics() const {
    // Evaluate system
    preComp_sys_->Request(Request::Dynamics, t, x_, u_);
    vector_t dx_sys = testSystem->ComputeFlowMap(t, x_sys_, u_sys_, *preComp_sys_);

    // Evaluate loopshaping system
    preComp_->Request(Request::Dynamics, t, x_, u_);
    vector_t dx = testLoopshapingDynamics->ComputeFlowMap(t, x_, u_, *preComp_);

    // System part of the flowmap should stay the same
    ASSERT_TRUE(dx_sys.isApprox(dx.head(dx_sys.rows())));
  }

  void EvaluateDynamicsApproximation() const {
    // Extract linearization
    preComp_->Request(Request::Dynamics + Request::Approximation, t, x_, u_);
    const auto linearization = testLoopshapingDynamics->LinearApproximation(t, x_, u_, *preComp_);

    // Reevaluate at disturbed state
    preComp_->Request(Request::Dynamics, t, x_ + x_disturbance_, u_ + u_disturbance_);
    vector_t dx_disturbance = testLoopshapingDynamics->ComputeFlowMap(t, x_ + x_disturbance_, u_ + u_disturbance_, *preComp_);

    // Evaluate approximation
    vector_t dx_approximation = linearization.f + linearization.dfdx * x_disturbance_ + linearization.dfdu * u_disturbance_;

    // Difference between new evaluation and linearization should be less than tol
    ASSERT_LE((dx_disturbance - dx_approximation).array().abs().maxCoeff(), tol);
  }

  void EvaluateJumpMap() const {
    // Evaluate jump map
    preComp_sys_->RequestPreJump(Request::Dynamics, t, x_sys_);
    const vector_t jumpMap_sys = testSystem->ComputeJumpMap(t, x_sys_, *preComp_sys_);

    preComp_->RequestPreJump(Request::Dynamics, t, x_);
    const vector_t jumpMap = testLoopshapingDynamics->ComputeJumpMap(t, x_, *preComp_);

    EXPECT_TRUE(jumpMap.head(x_sys_.rows()).isApprox(jumpMap_sys));
  }

  void EvaluateJumpMapApproximation() {
    // Evaluate linearization
    preComp_sys_->RequestPreJump(Request::Dynamics + Request::Approximation, t, x_sys_);
    const auto jumpMap_sys = testSystem->JumpMapLinearApproximation(t, x_sys_, *preComp_sys_);

    preComp_->RequestPreJump(Request::Dynamics + Request::Approximation, t, x_);
    const auto jumpMap = testLoopshapingDynamics->JumpMapLinearApproximation(t, x_, *preComp_);

    EXPECT_TRUE(jumpMap.f.head(x_sys_.rows()).isApprox(jumpMap_sys.f));
    EXPECT_TRUE(jumpMap.dfdx.topLeftCorner(x_sys_.rows(), x_sys_.rows()).isApprox(jumpMap_sys.dfdx));
    EXPECT_TRUE(jumpMap.dfdu.size() == 0);
    EXPECT_TRUE(jumpMap_sys.dfdu.size() == 0);
  }

 private:
  std::unique_ptr<LinearSystemDynamics> testSystem;
  std::unique_ptr<LoopshapingDynamics> testLoopshapingDynamics;
};

};  // namespace bolza
