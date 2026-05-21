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

#pragma once

#include "autonomy/control/controller/nmpc_controller/models/body_twist.hpp"
#include "autonomy/control/controller/nmpc_controller/models/kinematic_pose.hpp"

namespace autonomy {
namespace control {
namespace controller {
namespace nmpc {
namespace models {

/** Differential-drive parameters (track width = wheel separation, m). */
struct DifferentialDriveParams {
    double track_width{0.3};
};

using DifferentialDriveState = Pose2D;

/** Wheel linear velocities [v_left, v_right] (m/s). */
struct DifferentialDriveControl {
    double v_left{0.0};
    double v_right{0.0};
};

/** Forward kinematics: (v_l, v_r) -> (v, omega). */
BodyTwist ForwardKinematics(const DifferentialDriveControl& control,
                              double track_width);

/** Inverse kinematics: (v, omega) -> (v_l, v_r). */
DifferentialDriveControl InverseKinematics(const BodyTwist& twist,
                                           double track_width);

/** Discrete-time Euler integration of the diff-drive pose dynamics. */
Pose2D Propagate(const Pose2D& state, const DifferentialDriveControl& control,
                 double dt, double track_width);

void ClampWheelVelocities(DifferentialDriveControl& control,
                            double v_wheel_min, double v_wheel_max);

/** Wheel speed limits implied by body (v, omega) box constraints. */
void WheelLimitsFromBodyConstraints(double v_min, double v_max, double omega_min,
                                    double omega_max, double track_width,
                                    double* v_wheel_min, double* v_wheel_max);

}  // namespace models
}  // namespace nmpc
}  // namespace controller
}  // namespace control
}  // namespace autonomy
