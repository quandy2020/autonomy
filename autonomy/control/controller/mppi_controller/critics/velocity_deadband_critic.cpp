/*
 * Copyright 2025 The Openbot Authors (duyongquan)
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include "autonomy/control/controller/mppi_controller/critics/velocity_deadband_critic.hpp"

#include "autolink/common/log.hpp"

namespace autonomy {
namespace control {
namespace controller {
namespace mppi_controller {
namespace critics {

void VelocityDeadbandCritic::initialize() {
    if (!options_) {
        AWARN << "Options not set, using defaults";
        power_ = 1;
        weight_ = 35.0f;
        deadband_velocities_ = {0.05f, 0.05f, 0.05f};
        return;
    }

    // Load from proto options
    if (options_->has_velocity_deadband_critic()) {
        const auto& critic = options_->velocity_deadband_critic();
        enabled_ = critic.enabled();
        power_ = critic.cost_power();
        weight_ = static_cast<float>(critic.cost_weight());

        // Load deadband velocities
        deadband_velocities_.clear();
        for (int i = 0; i < critic.deadband_velocities_size() && i < 3; ++i) {
            deadband_velocities_.push_back(
                static_cast<float>(critic.deadband_velocities(i)));
        }
        // Ensure we have 3 values
        while (deadband_velocities_.size() < 3) {
            deadband_velocities_.push_back(0.05f);  // Default
        }
    } else {
        enabled_ = true;
        power_ = 1;
        weight_ = 35.0f;                               // Default
        deadband_velocities_ = {0.05f, 0.05f, 0.05f};  // Default
    }

    AINFO << "VelocityDeadbandCritic instantiated with " << power_ << " power, "
          << weight_ << " weight, deadband_velocity ["
          << deadband_velocities_.at(0) << "," << deadband_velocities_.at(1)
          << "," << deadband_velocities_.at(2) << "]";
}

void VelocityDeadbandCritic::score(CriticData& data) {
    if (!enabled_) {
        return;
    }

    if (data.motion_model->isHolonomic()) {
        if (power_ > 1u) {
            data.costs +=
                ((((fabs(deadband_velocities_[0]) - data.state.vx.abs())
                       .max(0.0f) +
                   (fabs(deadband_velocities_[1]) - data.state.vy.abs())
                       .max(0.0f) +
                   (fabs(deadband_velocities_[2]) - data.state.wz.abs())
                       .max(0.0f)) *
                  data.model_dt)
                     .rowwise()
                     .sum() *
                 weight_)
                    .pow(power_)
                    .eval();
        } else {
            data.costs +=
                ((((fabs(deadband_velocities_[0]) - data.state.vx.abs())
                       .max(0.0f) +
                   (fabs(deadband_velocities_[1]) - data.state.vy.abs())
                       .max(0.0f) +
                   (fabs(deadband_velocities_[2]) - data.state.wz.abs())
                       .max(0.0f)) *
                  data.model_dt)
                     .rowwise()
                     .sum() *
                 weight_)
                    .eval();
        }
        return;
    }

    if (power_ > 1u) {
        data.costs +=
            ((((fabs(deadband_velocities_[0]) - data.state.vx.abs()).max(0.0f) +
               (fabs(deadband_velocities_[2]) - data.state.wz.abs())
                   .max(0.0f)) *
              data.model_dt)
                 .rowwise()
                 .sum() *
             weight_)
                .pow(power_)
                .eval();
    } else {
        data.costs +=
            ((((fabs(deadband_velocities_[0]) - data.state.vx.abs()).max(0.0f) +
               (fabs(deadband_velocities_[2]) - data.state.wz.abs())
                   .max(0.0f)) *
              data.model_dt)
                 .rowwise()
                 .sum() *
             weight_)
                .eval();
    }
    return;
}

}  // namespace critics
}  // namespace mppi_controller
}  // namespace controller
}  // namespace control
}  // namespace autonomy

// Plugins
CLASS_LOADER_REGISTER_CLASS(
    autonomy::control::controller::mppi_controller::critics::
        VelocityDeadbandCritic,
    autonomy::control::controller::mppi_controller::CriticFunction)