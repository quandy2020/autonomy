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

 #include "autonomy/control/controller/mppi_controller/critics/constraint_critic.hpp"

 #include "autolink/common/log.hpp"
 
 namespace autonomy {
 namespace control {
 namespace controller {
 namespace mppi_controller {
 namespace critics {
 
 void ConstraintCritic::initialize() {
     if (!options_) {
         AWARN << "Options not set, using defaults";
         power_ = 1;
         weight_ = 4.0f;
         return;
     }
 
    // Load from proto options
    if (options_->has_constraint_critic()) {
        const auto& critic = options_->constraint_critic();
        enabled_ = critic.enabled();
        power_ = critic.cost_power();
        weight_ = critic.cost_weight();
    } else {
        enabled_ = true;
        power_ = 1;
        weight_ = 4.0f;  // Default
    }
 
     AINFO << "ConstraintCritic instantiated with " << power_ << " power and "
           << weight_ << " weight.";
 
     float vx_max, vy_max, vx_min;
     vx_max = options_->vx_max();
     vy_max = options_->vy_max();
     vx_min = options_->vx_min();
 
     const float min_sgn = vx_min > 0.0f ? 1.0f : -1.0f;
     max_vel_ = sqrtf(vx_max * vx_max + vy_max * vy_max);
     min_vel_ = min_sgn * sqrtf(vx_min * vx_min + vy_max * vy_max);
 }
 
 void ConstraintCritic::score(CriticData& data) {
     if (!enabled_) {
         return;
     }
 
     // Differential motion model
     auto diff = dynamic_cast<DiffDriveMotionModel*>(data.motion_model.get());
     if (diff != nullptr) {
         if (power_ > 1u) {
             data.costs += (((((data.state.vx - max_vel_).max(0.0f) +
                               (min_vel_ - data.state.vx).max(0.0f)) *
                              data.model_dt)
                                 .rowwise()
                                 .sum()
                                 .eval()) *
                            weight_)
                               .pow(power_)
                               .eval();
         } else {
             data.costs += (((((data.state.vx - max_vel_).max(0.0f) +
                               (min_vel_ - data.state.vx).max(0.0f)) *
                              data.model_dt)
                                 .rowwise()
                                 .sum()
                                 .eval()) *
                            weight_)
                               .eval();
         }
         return;
     }
 
     // Omnidirectional motion model
     auto omni = dynamic_cast<OmniMotionModel*>(data.motion_model.get());
     if (omni != nullptr) {
         auto& vx = data.state.vx;
         unsigned int n_rows = data.state.vx.rows();
         unsigned int n_cols = data.state.vx.cols();
         Eigen::ArrayXXf sgn(n_rows, n_cols);
         sgn = vx.unaryExpr([](const float x) { return copysignf(1.0f, x); });
 
         auto vel_total =
             sgn * (data.state.vx.square() + data.state.vy.square()).sqrt();
         if (power_ > 1u) {
             data.costs += ((((vel_total - max_vel_).max(0.0f) +
                              (min_vel_ - vel_total).max(0.0f)) *
                             data.model_dt)
                                .rowwise()
                                .sum()
                                .eval() *
                            weight_)
                               .pow(power_)
                               .eval();
         } else {
             data.costs += ((((vel_total - max_vel_).max(0.0f) +
                              (min_vel_ - vel_total).max(0.0f)) *
                             data.model_dt)
                                .rowwise()
                                .sum()
                                .eval() *
                            weight_)
                               .eval();
         }
         return;
     }
 
     // Ackermann motion model
     auto acker = dynamic_cast<AckermannMotionModel*>(data.motion_model.get());
     if (acker != nullptr) {
         auto& vx = data.state.vx;
         auto& wz = data.state.wz;
         float min_turning_rad = acker->getMinTurningRadius();
         auto out_of_turning_rad_motion =
             (min_turning_rad - (vx.abs() / wz.abs())).max(0.0f);
         if (power_ > 1u) {
             data.costs +=
                 ((((vx - max_vel_).max(0.0f) + (min_vel_ - vx).max(0.0f) +
                    out_of_turning_rad_motion) *
                   data.model_dt)
                      .rowwise()
                      .sum()
                      .eval() *
                  weight_)
                     .pow(power_)
                     .eval();
         } else {
             data.costs +=
                 ((((vx - max_vel_).max(0.0f) + (min_vel_ - vx).max(0.0f) +
                    out_of_turning_rad_motion) *
                   data.model_dt)
                      .rowwise()
                      .sum()
                      .eval() *
                  weight_)
                     .eval();
         }
         return;
     }
 }
 
 }  // namespace critics
 }  // namespace mppi_controller
 }  // namespace controller
 }  // namespace control
 }  // namespace autonomy
 