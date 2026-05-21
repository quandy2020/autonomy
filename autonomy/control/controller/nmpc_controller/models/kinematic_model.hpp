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
#include "autonomy/control/controller/nmpc_controller/models/differential_drive_model.hpp"
#include "autonomy/control/controller/nmpc_controller/models/kinematic_pose.hpp"
#include "autonomy/control/controller/nmpc_controller/models/quadruped_model.hpp"
#include "autonomy/control/controller/nmpc_controller/models/unicycle_model.hpp"
#include "autonomy/control/proto/nmpc_controller.pb.h"

namespace autonomy {
namespace control {
namespace controller {
namespace nmpc {
namespace models {

enum class KinematicModelType {
    kUnicycle,
    kDifferentialDrive,
    kQuadruped,
};

/** Two-dimensional control vector used by the shooting solver. */
struct Control2D {
    double u0{0.0};
    double u1{0.0};
};

/**
 * @brief Dispatches propagation / constraints across planar kinematic models.
 */
class KinematicModel
{
public:
    explicit KinematicModel(const proto::NmpcControllerOptions& options);

    KinematicModelType type() const { return type_; }
    bool IsHolonomicQuadruped() const;
    bool UsesBodyTwistOutput() const;

    Pose2D Propagate(const Pose2D& state, const Control2D& control,
                     double dt) const;

    void ProjectControl(Control2D& control) const;

    double ControlStagePenalty(const Control2D& control, double r_v,
                               double r_u1) const;

    BodyTwist ToBodyTwist(const Control2D& control) const;

    /** Box bounds for Control2D::u0 and u1 (solver decision variables). */
    void GetControlBounds(double* u0_lower, double* u0_upper, double* u1_lower,
                          double* u1_upper) const;

    static KinematicModelType ParseType(const std::string& name);

private:
    KinematicModelType type_{KinematicModelType::kUnicycle};
    QuadrupedParams quadruped_params_;
    double track_width_{0.3};
    double v_min_{0.0};
    double v_max_{0.5};
    double omega_min_{-1.0};
    double omega_max_{1.0};
    double wheel_v_min_{-0.5};
    double wheel_v_max_{0.5};
    double r_vy_{0.1};
};

}  // namespace models
}  // namespace nmpc
}  // namespace controller
}  // namespace control
}  // namespace autonomy
