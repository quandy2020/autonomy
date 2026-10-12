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

#include <bolza/core/dynamics/system_dynamics_base_ad.hpp>

#include "examples/cartpole/cart_pole_parameters.hpp"
#include "examples/cartpole/cartpole.hpp"

namespace bolza {
namespace cartpole {

/**
 * CartPole dynamics.
 * refer to: https://pdfs.semanticscholar.org/f95b/9d4cc0814034f2e601cb91fcd70b2e806420.pdf
 */
class CartPoleSytemDynamics : public SystemDynamicsBaseAD {
 public:
  CartPoleSytemDynamics(const CartPoleParameters& cartPoleParameters, const std::string& libraryFolder, bool verbose)
      : param_(cartPoleParameters) {
    Initialize(STATE_DIM, INPUT_DIM, "cartpole_dynamics", libraryFolder, true, verbose);
  }

  ~CartPoleSytemDynamics() override = default;

  CartPoleSytemDynamics(const CartPoleSytemDynamics& rhs) = default;

  CartPoleSytemDynamics* Clone() const override { return new CartPoleSytemDynamics(*this); }

  ad_vector_t SystemFlowMap(ad_scalar_t time, const ad_vector_t& state, const ad_vector_t& input,
                            const ad_vector_t& parameters) const override {
    const ad_scalar_t cosTheta = cos(state(0));
    const ad_scalar_t sinTheta = sin(state(0));

    // Inertia tensor
    Eigen::Matrix<ad_scalar_t, 2, 2> I;
    I << static_cast<ad_scalar_t>(param_.poleSteinerMoi_), static_cast<ad_scalar_t>(param_.poleMass_ * param_.poleHalfLength_ * cosTheta),
        static_cast<ad_scalar_t>(param_.poleMass_ * param_.poleHalfLength_ * cosTheta),
        static_cast<ad_scalar_t>(param_.cartMass_ + param_.poleMass_);

    // RHS
    Eigen::Matrix<ad_scalar_t, 2, 1> rhs(
        param_.poleMass_ * param_.poleHalfLength_ * param_.gravity_ * sinTheta,
        input(0) + param_.poleMass_ * param_.poleHalfLength_ * pow(state(2), 2) * sinTheta);

    // dxdt
    ad_vector_t stateDerivative(STATE_DIM);
    stateDerivative << state.tail<2>(), I.inverse() * rhs;
    return stateDerivative;
  }

 private:
  CartPoleParameters param_;
};

}  // namespace cartpole
}  // namespace bolza
