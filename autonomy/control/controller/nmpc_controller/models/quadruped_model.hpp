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

#include <string>

#include "autonomy/control/controller/nmpc_controller/models/body_twist.hpp"
#include "autonomy/control/controller/nmpc_controller/models/kinematic_pose.hpp"

namespace autonomy {
namespace control {
namespace controller {
namespace nmpc {
namespace models {

/** High-level gait abstraction for planar quadruped MPC (no leg IK). */
enum class QuadrupedGaitMode {
    kTrotting,
    kHolonomic,
};

using QuadrupedState = Pose2D;

/** Trotting / heading-following: [v, omega] (m/s, rad/s). */
struct QuadrupedTrottingControl {
    double v{0.0};
    double omega{0.0};
};

/** Holonomic crawl / crab: body-frame [vx, vy] (m/s). */
struct QuadrupedHolonomicControl {
    double vx{0.0};
    double vy{0.0};
};

struct QuadrupedParams {
    QuadrupedGaitMode gait{QuadrupedGaitMode::kTrotting};
    double v_min{0.0};
    double v_max{0.5};
    double omega_min{-1.0};
    double omega_max{1.0};
    /** Max |vy| for holonomic gait (m/s). */
    double max_lateral_velocity{0.25};
    /** Min |v|/|omega| when turning (m); 0 disables coupling. */
    double min_turn_radius{0.0};
};

QuadrupedGaitMode ParseQuadrupedGait(const std::string& name);

/** Trotting: nonholonomic planar integration (same structure as unicycle). */
Pose2D PropagateTrotting(const Pose2D& state,
                         const QuadrupedTrottingControl& control, double dt);

/** Holonomic: integrate body-frame (vx, vy) without yaw rate in control. */
Pose2D PropagateHolonomic(const Pose2D& state,
                          const QuadrupedHolonomicControl& control, double dt);

void ProjectTrottingControl(QuadrupedTrottingControl& control,
                            const QuadrupedParams& params);

void ProjectHolonomicControl(QuadrupedHolonomicControl& control,
                             const QuadrupedParams& params);

BodyTwist TrottingToBodyTwist(const QuadrupedTrottingControl& control);

BodyTwist HolonomicToBodyTwist(const QuadrupedHolonomicControl& control);

double TrottingControlPenalty(const QuadrupedTrottingControl& control,
                              double r_v, double r_omega);

double HolonomicControlPenalty(const QuadrupedHolonomicControl& control,
                               double r_v, double r_vy);

}  // namespace models
}  // namespace nmpc
}  // namespace controller
}  // namespace control
}  // namespace autonomy
