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

#include <bolza/core/types.hpp>
#include <bolza/core/dynamics/controlled_system_base.hpp>

namespace bolza {

class ballDyn : public ControlledSystemBase {
 public:
  BallDyn() = default;
  ~BallDyn() = default;

  vector_t ComputeFlowMap(scalar_t t, const vector_t& x, const vector_t& u, const PreComputation&) override {
    matrix_t A(2, 2);
    A << 0.0, 1.0, 0.0, 0.0;
    vector_t F(2, 1);
    F << 0.0, -9.81;
    return A * x + F;
  }

  vector_t ComputeJumpMap(scalar_t t, const vector_t& x, const PreComputation&) override {
    vector_t MappedState(2);
    mappedState[0] = x[0];
    mappedState[1] = -0.95 * x[1];
    return mappedState;
  }

  vector_t ComputeGuardSurfaces(scalar_t t, const vector_t& x) override {
    vector_t GuardSurfaces(2);
    guardSurfaces[0] = x[0];
    guardSurfaces[1] = -x[0] + 0.5;
    return guardSurfaces;
  }

  ballDyn* Clone() const override { return new BallDyn(*this); }
};
}  // namespace bolza
