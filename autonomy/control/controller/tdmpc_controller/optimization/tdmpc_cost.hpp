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

#include "autonomy/control/controller/nmpc_controller/optimization/optimization_guard.hpp"
#include "autonomy/control/controller/nmpc_controller/models/kinematic_pose.hpp"
#include "autonomy/control/controller/tdmpc_controller/models_alias.hpp"
#include "autonomy/control/controller/tdmpc_controller/tracking/contouring_reference.hpp"
#include "autonomy/control/controller/tdmpc_controller/tracking/path_spline.hpp"
#include "autonomy/control/proto/tdmpc_controller.pb.h"

namespace autonomy {
namespace map {
namespace costmap_2d {
class Costmap2DWrapper;
}  // namespace costmap_2d
}  // namespace map

namespace control {
namespace controller {
namespace tdmpc {
namespace mpc_opt {

using models::Pose2D;

struct TdmpcCostWeights {
    double q_contour{10.0};
    double q_lag{10.0};
    double q_yaw{5.0};
    double q_velocity{1.0};
    double q_contour_terminal{0.0};
    double q_lag_terminal{0.0};
    double q_yaw_terminal{0.0};
    double r_v{0.1};
    double r_omega{0.1};
    double r_du{0.0};
    double q_obstacle{50.0};
    double obstacle_cost_threshold{200.0};
};

TdmpcCostWeights WeightsFromOptions(const proto::TdmpcControllerOptions& options);

double ContouringStageCost(const Pose2D& state, const tracking::PathSpline& spline,
                           double ref_arc_length, const TdmpcCostWeights& weights);

double ContouringTerminalCost(const Pose2D& state,
                              const tracking::PathSpline& spline,
                              double ref_arc_length,
                              const TdmpcCostWeights& weights);

double ObstacleStageCost(const Pose2D& state,
                         const map::costmap_2d::Costmap2DWrapper* costmap,
                         const TdmpcCostWeights& weights, bool enabled);

}  // namespace mpc_opt
}  // namespace tdmpc
}  // namespace controller
}  // namespace control
}  // namespace autonomy
