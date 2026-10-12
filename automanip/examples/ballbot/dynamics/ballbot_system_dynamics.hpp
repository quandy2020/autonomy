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

// ocs2
#include <bolza/core/dynamics/system_dynamics_base_ad.hpp>

// ballbot
#include "examples/ballbot/ballbot.hpp"
#include "examples/ballbot/ballbot_parameters.hpp"

namespace bolza {
namespace ballbot {

/**
 * BallbotSystemDynamics class
 * This class implements the dynamics for the ballbot.
 * The equations of motion are generated through robcogen, using the following set of generalized coordinates:
 * (ballPosition_x, ballPosition_y, eulerAnglesZyx theta_z, eulerAnglesZyx theta_y, eulerAnglesZyx theta_x)
 * The control input are u = (torque_wheel1, torque_wheel2, torque_wheel3)
 */
class BallbotSystemDynamics : public SystemDynamicsBaseAD {
 public:
  /** Constructor */
  BallbotSystemDynamics(const std::string& libraryFolder, bool recompileLibraries) : SystemDynamicsBaseAD() {
    wheelRadius_ = param_.wheelRadius_;
    ballRadius_ = param_.ballRadius_;

    Initialize(STATE_DIM, INPUT_DIM, "ballbot_dynamics", libraryFolder, recompileLibraries, true);
  }

  /** Destructor */
  ~BallbotSystemDynamics() override = default;

  BallbotSystemDynamics(const BallbotSystemDynamics& rhs) = default;

  BallbotSystemDynamics* Clone() const override { return new BallbotSystemDynamics(*this); }

  ad_vector_t SystemFlowMap(ad_scalar_t time, const ad_vector_t& state, const ad_vector_t& input,
                            const ad_vector_t& parameters) const override;

 private:
  BallbotParameters param_;
  scalar_t wheelRadius_;
  scalar_t ballRadius_;
};

}  // namespace ballbot
}  // namespace bolza
