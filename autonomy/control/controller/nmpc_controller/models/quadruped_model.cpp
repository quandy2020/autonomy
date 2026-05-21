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

#include "autonomy/control/controller/nmpc_controller/models/quadruped_model.hpp"

#include <algorithm>
#include <cctype>
#include <cmath>
#include <string>

namespace autonomy {
namespace control {
namespace controller {
namespace nmpc {
namespace models {

namespace {

std::string ToLower(std::string s) {
    for (char& c : s) {
        c = static_cast<char>(std::tolower(static_cast<unsigned char>(c)));
    }
    return s;
}

void EnforceMinTurnRadius(double min_turn_radius, double* v, double* omega) {
    if (min_turn_radius <= 1e-6 || std::abs(*omega) < 1e-6) {
        return;
    }
    const double v_min_for_turn = std::abs(*omega) * min_turn_radius;
    if (std::abs(*v) < v_min_for_turn) {
        *v = std::copysign(v_min_for_turn, *v != 0.0 ? *v : *omega);
    }
}

}  // namespace

QuadrupedGaitMode ParseQuadrupedGait(const std::string& name) {
    const std::string key = ToLower(name);
    if (key.find("holo") != std::string::npos || key.find("crab") != std::string::npos ||
        key.find("omni") != std::string::npos || key == "lateral") {
        return QuadrupedGaitMode::kHolonomic;
    }
    return QuadrupedGaitMode::kTrotting;
}

Pose2D PropagateTrotting(const Pose2D& state,
                         const QuadrupedTrottingControl& control, double dt) {
    Pose2D next = state;
    next.x += control.v * std::cos(state.theta) * dt;
    next.y += control.v * std::sin(state.theta) * dt;
    next.theta += control.omega * dt;
    return next;
}

Pose2D PropagateHolonomic(const Pose2D& state,
                          const QuadrupedHolonomicControl& control, double dt) {
    const double c = std::cos(state.theta);
    const double s = std::sin(state.theta);
    Pose2D next = state;
    next.x += (c * control.vx - s * control.vy) * dt;
    next.y += (s * control.vx + c * control.vy) * dt;
    return next;
}

void ProjectTrottingControl(QuadrupedTrottingControl& control,
                            const QuadrupedParams& params) {
    const double v_lo = std::min(params.v_min, params.v_max);
    const double v_hi = std::max(params.v_min, params.v_max);
    const double omega_lo = std::min(params.omega_min, params.omega_max);
    const double omega_hi = std::max(params.omega_min, params.omega_max);

    control.v = std::clamp(control.v, v_lo, v_hi);
    control.omega = std::clamp(control.omega, omega_lo, omega_hi);
    EnforceMinTurnRadius(params.min_turn_radius, &control.v, &control.omega);
}

void ProjectHolonomicControl(QuadrupedHolonomicControl& control,
                             const QuadrupedParams& params) {
    const double v_lo = std::min(params.v_min, params.v_max);
    const double v_hi = std::max(params.v_min, params.v_max);
    const double vy_lim = params.max_lateral_velocity > 0.0
                              ? params.max_lateral_velocity
                              : v_hi;

    control.vx = std::clamp(control.vx, v_lo, v_hi);
    control.vy = std::clamp(control.vy, -vy_lim, vy_lim);

    const double speed = std::hypot(control.vx, control.vy);
    if (speed > v_hi && speed > 1e-9) {
        const double scale = v_hi / speed;
        control.vx *= scale;
        control.vy *= scale;
    }
}

BodyTwist TrottingToBodyTwist(const QuadrupedTrottingControl& control) {
    BodyTwist twist;
    twist.v = control.v;
    twist.vy = 0.0;
    twist.omega = control.omega;
    return twist;
}

BodyTwist HolonomicToBodyTwist(const QuadrupedHolonomicControl& control) {
    BodyTwist twist;
    twist.v = control.vx;
    twist.vy = control.vy;
    twist.omega = 0.0;
    return twist;
}

double TrottingControlPenalty(const QuadrupedTrottingControl& control,
                              double r_v, double r_omega) {
    return r_v * control.v * control.v +
           r_omega * control.omega * control.omega;
}

double HolonomicControlPenalty(const QuadrupedHolonomicControl& control,
                               double r_v, double r_vy) {
    return r_v * control.vx * control.vx + r_vy * control.vy * control.vy;
}

}  // namespace models
}  // namespace nmpc
}  // namespace controller
}  // namespace control
}  // namespace autonomy
