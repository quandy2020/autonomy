/*
 * Copyright 2025 The Openbot Authors
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

#include "autonomy/vehicle/motion/kinematics_control.hpp"

#include <algorithm>

namespace autonomy {
namespace vehicle {
namespace motion {

using ::autonomy::vehicle::proto::KinematicsControlCommand;
using ::autonomy::vehicle::proto::VehicleModel;

KinematicsControl::KinematicsControl(const VehicleModel& model) : model_(model) {}

void KinematicsControl::ApplyLimits(KinematicsControlCommand* cmd) const {
    if (!cmd) {
        return;
    }

    auto* vel = cmd->mutable_velocity();
    auto* acc = cmd->mutable_acceleration();

    const double max_v_fwd = model_.max_linear_speed();
    const double max_v_rev = model_.max_reverse_speed();
    const double max_w = model_.max_angular_speed();
    const double max_a = model_.max_linear_acceleration();
    const double max_aw = model_.max_angular_acceleration();

    // 线速度限幅（假定主要使用 linear.x 作为前进/后退方向）
    if (vel) {
        auto* lin = vel->mutable_linear();
        auto* ang = vel->mutable_angular();

        if (lin) {
            double vx = lin->x();
            // forward limit
            if (max_v_fwd > 0.0) {
                vx = std::min(vx, max_v_fwd);
            }
            // reverse limit
            if (max_v_rev > 0.0) {
                vx = std::max(vx, -max_v_rev);
            }
            lin->set_x(vx);
        }

        // 角速度限幅（假定主要使用 angular.z 作为航向角速度）
        if (ang && max_w > 0.0) {
            double wz = ang->z();
            wz = std::clamp(wz, -max_w, max_w);
            ang->set_z(wz);
        }
    }

    // 线加速度 / 角加速度限幅
    if (acc) {
        auto* lin_a = acc->mutable_linear();
        auto* ang_a = acc->mutable_angular();

        if (lin_a && max_a > 0.0) {
            double ax = lin_a->x();
            ax = std::clamp(ax, -max_a, max_a);
            lin_a->set_x(ax);
        }

        if (ang_a && max_aw > 0.0) {
            double awz = ang_a->z();
            awz = std::clamp(awz, -max_aw, max_aw);
            ang_a->set_z(awz);
        }
    }
}

}  // namespace motion
}  // namespace vehicle
}  // namespace autonomy
