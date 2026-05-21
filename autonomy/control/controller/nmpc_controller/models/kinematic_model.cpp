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

#include "autonomy/control/controller/nmpc_controller/models/kinematic_model.hpp"

#include <algorithm>
#include <cctype>
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

QuadrupedParams BuildQuadrupedParams(
    const proto::NmpcControllerOptions& options) {
    QuadrupedParams params;
    params.gait = ParseQuadrupedGait(options.quadruped_gait());
    params.v_min = options.allow_backward() ? options.v_min()
                                            : std::max(options.v_min(), 0.0);
    params.v_max = options.v_max() > params.v_min ? options.v_max() : 0.5;
    params.omega_min = options.omega_min();
    params.omega_max = options.omega_max() > options.omega_min()
                           ? options.omega_max()
                           : 1.0;
    params.max_lateral_velocity =
        options.max_lateral_velocity() > 0.0 ? options.max_lateral_velocity()
                                               : 0.25;
    params.min_turn_radius =
        options.min_turn_radius() > 0.0 ? options.min_turn_radius() : 0.0;
    return params;
}

}  // namespace

KinematicModelType KinematicModel::ParseType(const std::string& name) {
    const std::string key = ToLower(name);
    if (key.find("quad") != std::string::npos ||
        key.find("legged") != std::string::npos ||
        key.find("four_leg") != std::string::npos || key == "4leg") {
        return KinematicModelType::kQuadruped;
    }
    if (key.find("diff") != std::string::npos ||
        key.find("differential") != std::string::npos ||
        key == "diff_drive" || key == "differential_drive") {
        return KinematicModelType::kDifferentialDrive;
    }
    return KinematicModelType::kUnicycle;
}

KinematicModel::KinematicModel(const proto::NmpcControllerOptions& options)
    : type_(ParseType(options.kinematic_model())),
      quadruped_params_(BuildQuadrupedParams(options)),
      track_width_(options.track_width() > 1e-6 ? options.track_width() : 0.3),
      v_min_(options.allow_backward() ? options.v_min()
                                      : std::max(options.v_min(), 0.0)),
      v_max_(options.v_max() > v_min_ ? options.v_max() : 0.5),
      omega_min_(options.omega_min()),
      omega_max_(options.omega_max() > options.omega_min() ? options.omega_max()
                                                           : 1.0),
      r_vy_(options.r_vy() > 0.0 ? options.r_vy() : options.r_omega()) {
    WheelLimitsFromBodyConstraints(v_min_, v_max_, omega_min_, omega_max_,
                                   track_width_, &wheel_v_min_, &wheel_v_max_);
}

bool KinematicModel::IsHolonomicQuadruped() const {
    return type_ == KinematicModelType::kQuadruped &&
           quadruped_params_.gait == QuadrupedGaitMode::kHolonomic;
}

bool KinematicModel::UsesBodyTwistOutput() const {
    return IsHolonomicQuadruped();
}

Pose2D KinematicModel::Propagate(const Pose2D& state, const Control2D& control,
                                 double dt) const {
    if (type_ == KinematicModelType::kQuadruped) {
        if (quadruped_params_.gait == QuadrupedGaitMode::kHolonomic) {
            QuadrupedHolonomicControl holo;
            holo.vx = control.u0;
            holo.vy = control.u1;
            return PropagateHolonomic(state, holo, dt);
        }
        QuadrupedTrottingControl trot;
        trot.v = control.u0;
        trot.omega = control.u1;
        return PropagateTrotting(state, trot, dt);
    }
    if (type_ == KinematicModelType::kDifferentialDrive) {
        DifferentialDriveControl wheels;
        wheels.v_left = control.u0;
        wheels.v_right = control.u1;
        return models::Propagate(state, wheels, dt, track_width_);
    }
    UnicycleControl unicycle;
    unicycle.v = control.u0;
    unicycle.omega = control.u1;
    return models::Propagate(state, unicycle, dt);
}

void KinematicModel::ProjectControl(Control2D& control) const {
    if (type_ == KinematicModelType::kQuadruped) {
        if (quadruped_params_.gait == QuadrupedGaitMode::kHolonomic) {
            QuadrupedHolonomicControl holo;
            holo.vx = control.u0;
            holo.vy = control.u1;
            ProjectHolonomicControl(holo, quadruped_params_);
            control.u0 = holo.vx;
            control.u1 = holo.vy;
            return;
        }
        QuadrupedTrottingControl trot;
        trot.v = control.u0;
        trot.omega = control.u1;
        ProjectTrottingControl(trot, quadruped_params_);
        control.u0 = trot.v;
        control.u1 = trot.omega;
        return;
    }
    if (type_ == KinematicModelType::kDifferentialDrive) {
        DifferentialDriveControl wheels;
        wheels.v_left = control.u0;
        wheels.v_right = control.u1;
        ClampWheelVelocities(wheels, wheel_v_min_, wheel_v_max_);
        control.u0 = wheels.v_left;
        control.u1 = wheels.v_right;
        return;
    }
    control.u0 = std::clamp(control.u0, v_min_, v_max_);
    control.u1 = std::clamp(control.u1, omega_min_, omega_max_);
}

double KinematicModel::ControlStagePenalty(const Control2D& control, double r_v,
                                           double r_u1) const {
    if (type_ == KinematicModelType::kQuadruped) {
        if (quadruped_params_.gait == QuadrupedGaitMode::kHolonomic) {
            QuadrupedHolonomicControl holo;
            holo.vx = control.u0;
            holo.vy = control.u1;
            return HolonomicControlPenalty(holo, r_v, r_vy_);
        }
        QuadrupedTrottingControl trot;
        trot.v = control.u0;
        trot.omega = control.u1;
        return TrottingControlPenalty(trot, r_v, r_u1);
    }
    const BodyTwist twist = ToBodyTwist(control);
    if (type_ == KinematicModelType::kDifferentialDrive) {
        return r_v * twist.v * twist.v + r_u1 * twist.omega * twist.omega;
    }
    return r_v * twist.v * twist.v + r_u1 * twist.omega * twist.omega;
}

void KinematicModel::GetControlBounds(double* u0_lower, double* u0_upper,
                                      double* u1_lower, double* u1_upper) const {
    if (type_ == KinematicModelType::kDifferentialDrive) {
        *u0_lower = wheel_v_min_;
        *u0_upper = wheel_v_max_;
        *u1_lower = wheel_v_min_;
        *u1_upper = wheel_v_max_;
        return;
    }
    if (type_ == KinematicModelType::kQuadruped &&
        quadruped_params_.gait == QuadrupedGaitMode::kHolonomic) {
        const double v_lo = std::min(quadruped_params_.v_min, quadruped_params_.v_max);
        const double v_hi = std::max(quadruped_params_.v_min, quadruped_params_.v_max);
        const double vy_lim = quadruped_params_.max_lateral_velocity > 0.0
                                  ? quadruped_params_.max_lateral_velocity
                                  : v_hi;
        *u0_lower = v_lo;
        *u0_upper = v_hi;
        *u1_lower = -vy_lim;
        *u1_upper = vy_lim;
        return;
    }
    const double v_lo = std::min(v_min_, v_max_);
    const double v_hi = std::max(v_min_, v_max_);
    const double omega_lo = std::min(omega_min_, omega_max_);
    const double omega_hi = std::max(omega_min_, omega_max_);
    *u0_lower = v_lo;
    *u0_upper = v_hi;
    *u1_lower = omega_lo;
    *u1_upper = omega_hi;
}

BodyTwist KinematicModel::ToBodyTwist(const Control2D& control) const {
    if (type_ == KinematicModelType::kQuadruped) {
        if (quadruped_params_.gait == QuadrupedGaitMode::kHolonomic) {
            QuadrupedHolonomicControl holo;
            holo.vx = control.u0;
            holo.vy = control.u1;
            return HolonomicToBodyTwist(holo);
        }
        QuadrupedTrottingControl trot;
        trot.v = control.u0;
        trot.omega = control.u1;
        return TrottingToBodyTwist(trot);
    }
    if (type_ == KinematicModelType::kDifferentialDrive) {
        DifferentialDriveControl wheels;
        wheels.v_left = control.u0;
        wheels.v_right = control.u1;
        return ForwardKinematics(wheels, track_width_);
    }
    BodyTwist twist;
    twist.v = control.u0;
    twist.vy = 0.0;
    twist.omega = control.u1;
    return twist;
}

}  // namespace models
}  // namespace nmpc
}  // namespace controller
}  // namespace control
}  // namespace autonomy
