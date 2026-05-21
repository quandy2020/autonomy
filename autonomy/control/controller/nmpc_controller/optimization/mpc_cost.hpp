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

#include "autonomy/control/controller/nmpc_controller/models/kinematic_pose.hpp"
#include "autonomy/control/proto/nmpc_controller.pb.h"

namespace autonomy {
namespace control {
namespace controller {
namespace nmpc {
namespace mpc_opt {

using models::Pose2D;

struct MpcCostWeights {
    double q_x{10.0};
    double q_y{10.0};
    double q_yaw{5.0};
    double q_x_terminal{0.0};
    double q_y_terminal{0.0};
    double q_yaw_terminal{0.0};
    double r_v{0.1};
    double r_omega{0.1};
    double r_du{0.0};
};

MpcCostWeights WeightsFromOptions(const proto::NmpcControllerOptions& options);

double PoseStageCost(const Pose2D& state, const Pose2D& reference,
                    const MpcCostWeights& weights);

double TerminalCost(const Pose2D& state, const Pose2D& reference,
                    const MpcCostWeights& weights);

}  // namespace mpc_opt
}  // namespace nmpc
}  // namespace controller
}  // namespace control
}  // namespace autonomy
