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

#include <cmath>

#include <bolza/core/dynamics/controlled_system_base.hpp>

namespace bolza {

class pendulum_dyn final : public ControlledSystemBase {
 public:
  pendulum_dyn() = default;
  ~pendulum_dyn() override = default;

  vector_t ComputeFlowMap(scalar_t t, const vector_t& x, const vector_t& u, const PreComputation&) override {
    const double g = 9.81;
    const double L = 1;
    vector_t Dxdt(2);
    Dxdt << x[1], -(g / L) * std::sin(x[0]);
    return dxdt;
  }

  vector_t ComputeJumpMap(scalar_t t, const vector_t& state, const PreComputation&) override {
    vector_t MappedState(2);
    mappedState[0] = state[0];
    mappedState[1] = -0.9 * state[1];
    return mappedState;
  }

  vector_t ComputeGuardSurfaces(scalar_t t, const vector_t& state) override {
    vector_t GuardSurfaces(1);
    guardSurfaces[0] = state[0];
    return guardSurfaces;
  }

  pendulum_dyn* Clone() const override { return new pendulum_dyn(*this); }
};

}  // namespace bolza
