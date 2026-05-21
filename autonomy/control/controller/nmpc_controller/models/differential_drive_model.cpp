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

#include "autonomy/control/controller/nmpc_controller/models/differential_drive_model.hpp"

#include <algorithm>
#include <cmath>

namespace autonomy {
namespace control {
namespace controller {
namespace nmpc {
namespace models {

namespace {

double EffectiveTrackWidth(double track_width) {
    return track_width > 1e-6 ? track_width : 0.3;
}

}  // namespace

BodyTwist ForwardKinematics(const DifferentialDriveControl& control,
                              double track_width) {
    const double b = EffectiveTrackWidth(track_width);
    BodyTwist twist;
    twist.v = 0.5 * (control.v_left + control.v_right);
    twist.vy = 0.0;
    twist.omega = (control.v_right - control.v_left) / b;
    return twist;
}

DifferentialDriveControl InverseKinematics(const BodyTwist& twist,
                                           double track_width) {
    const double half_b = 0.5 * EffectiveTrackWidth(track_width);
    DifferentialDriveControl control;
    control.v_left = twist.v - half_b * twist.omega;
    control.v_right = twist.v + half_b * twist.omega;
    return control;
}

Pose2D Propagate(const Pose2D& state, const DifferentialDriveControl& control,
                 double dt, double track_width) {
    const BodyTwist twist = ForwardKinematics(control, track_width);
    Pose2D next = state;
    next.x += twist.v * std::cos(state.theta) * dt;
    next.y += twist.v * std::sin(state.theta) * dt;
    next.theta += twist.omega * dt;
    return next;
}

void ClampWheelVelocities(DifferentialDriveControl& control,
                            double v_wheel_min, double v_wheel_max) {
    control.v_left =
        std::clamp(control.v_left, v_wheel_min, v_wheel_max);
    control.v_right =
        std::clamp(control.v_right, v_wheel_min, v_wheel_max);
}

void WheelLimitsFromBodyConstraints(double v_min, double v_max, double omega_min,
                                    double omega_max, double track_width,
                                    double* v_wheel_min, double* v_wheel_max) {
    const double half_b = 0.5 * EffectiveTrackWidth(track_width);
    const double omega_lo = std::min(omega_min, omega_max);
    const double omega_hi = std::max(omega_min, omega_max);
    const double v_lo = std::min(v_min, v_max);
    const double v_hi = std::max(v_min, v_max);

    const double v_left_min = v_lo - half_b * omega_hi;
    const double v_left_max = v_hi - half_b * omega_lo;
    const double v_right_min = v_lo + half_b * omega_lo;
    const double v_right_max = v_hi + half_b * omega_hi;
    *v_wheel_min = std::min(v_left_min, v_right_min);
    *v_wheel_max = std::max(v_left_max, v_right_max);
}

}  // namespace models
}  // namespace nmpc
}  // namespace controller
}  // namespace control
}  // namespace autonomy
