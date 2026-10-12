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

#include <bolza/core/automatic_differentiation/types.hpp>
#include <bolza/core/dynamics/system_dynamics_base_ad.hpp>

namespace bolza {

class LinearSystemDynamicsAD : public SystemDynamicsBaseAD {
 public:
  LinearSystemDynamicsAD(const matrix_t& A, const matrix_t& B, const matrix_t& G) : SystemDynamicsBaseAD(), A_(A), B_(B), G_(G) {}

  LinearSystemDynamicsAD(const LinearSystemDynamicsAD& rhs) : SystemDynamicsBaseAD(rhs), A_(rhs.A_), B_(rhs.B_), G_(rhs.G_) {}

  ~LinearSystemDynamicsAD() override = default;

  LinearSystemDynamicsAD* Clone() const override { return new LinearSystemDynamicsAD(*this); }

 protected:
  ad_vector_t SystemFlowMap(ad_scalar_t time, const ad_vector_t& state, const ad_vector_t& input,
                            const ad_vector_t& parameters) const override {
    return A_.cast<ad_scalar_t>() * state + B_.cast<ad_scalar_t>() * input;
  }

  ad_vector_t SystemJumpMap(ad_scalar_t time, const ad_vector_t& state, const ad_vector_t& parameters) const override {
    return G_.cast<ad_scalar_t>() * state;
  }

 private:
  matrix_t A_;
  matrix_t B_;
  matrix_t G_;
};

}  // namespace bolza
