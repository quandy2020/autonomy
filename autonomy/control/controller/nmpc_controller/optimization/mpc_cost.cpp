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

#include "autonomy/control/controller/nmpc_controller/optimization/mpc_cost.hpp"

#include <cmath>

#include "autonomy/common/math/math.hpp"

namespace autonomy {
namespace control {
namespace controller {
namespace nmpc {
namespace mpc_opt {
namespace {

double SquaredYawError(double theta, double ref_theta) {
    const double err =
        ::autonomy::common::NormalizeAngleDifference(theta - ref_theta);
    return err * err;
}

}  // namespace

MpcCostWeights WeightsFromOptions(const proto::NmpcControllerOptions& options) {
    MpcCostWeights w;
    w.q_x = options.q_x() > 0.0 ? options.q_x() : 10.0;
    w.q_y = options.q_y() > 0.0 ? options.q_y() : 10.0;
    w.q_yaw = options.q_yaw() > 0.0 ? options.q_yaw() : 5.0;
    w.q_x_terminal = options.q_x_terminal();
    w.q_y_terminal = options.q_y_terminal();
    w.q_yaw_terminal = options.q_yaw_terminal();
    w.r_v = options.r_v() > 0.0 ? options.r_v() : 0.1;
    w.r_omega = options.r_omega() > 0.0 ? options.r_omega() : 0.1;
    w.r_du = options.r_du() > 0.0 ? options.r_du() : 0.0;
    return w;
}

double PoseStageCost(const Pose2D& state, const Pose2D& reference,
                     const MpcCostWeights& weights) {
    const double dx = state.x - reference.x;
    const double dy = state.y - reference.y;
    return weights.q_x * dx * dx + weights.q_y * dy * dy +
           weights.q_yaw * SquaredYawError(state.theta, reference.theta);
}

double TerminalCost(const Pose2D& state, const Pose2D& reference,
                    const MpcCostWeights& weights) {
    const double qx =
        weights.q_x_terminal > 0.0 ? weights.q_x_terminal : weights.q_x;
    const double qy =
        weights.q_y_terminal > 0.0 ? weights.q_y_terminal : weights.q_y;
    const double qyaw = weights.q_yaw_terminal > 0.0 ? weights.q_yaw_terminal
                                                     : weights.q_yaw;
    const double dx = state.x - reference.x;
    const double dy = state.y - reference.y;
    return qx * dx * dx + qy * dy * dy +
           qyaw * SquaredYawError(state.theta, reference.theta);
}

}  // namespace mpc_opt
}  // namespace nmpc
}  // namespace controller
}  // namespace control
}  // namespace autonomy
