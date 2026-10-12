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

#include <bolza/core/dynamics/system_dynamics_base.hpp>

#include "examples/quadrotor/quadrotor.hpp"
#include "examples/quadrotor/quadrotor_parameters.hpp"

namespace bolza {
namespace quadrotor {

class QuadrotorSystemDynamics final : public SystemDynamicsBase {
 public:
  /**
   * Constructor
   *
   * @param quadrotorParameters: Quadrotor parameters.
   */
  explicit QuadrotorSystemDynamics(QuadrotorParameters quadrotorParameters) : param_(std::move(quadrotorParameters)) {}

  /** Destructor */
  ~QuadrotorSystemDynamics() override = default;

  QuadrotorSystemDynamics* Clone() const override { return new QuadrotorSystemDynamics(*this); }

  vector_t ComputeFlowMap(scalar_t time, const vector_t& state, const vector_t& input, const PreComputation&) override;
  VectorFunctionLinearApproximation LinearApproximation(scalar_t t, const vector_t& x, const vector_t& u, const PreComputation&) override;

 private:
  QuadrotorParameters param_;

  matrix_t jacobianOfAngularVelocityMapping_;
};

}  // namespace quadrotor
}  // namespace bolza
