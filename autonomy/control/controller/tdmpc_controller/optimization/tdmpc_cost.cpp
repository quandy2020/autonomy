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

#include "autonomy/control/controller/tdmpc_controller/optimization/tdmpc_cost.hpp"

#include "autonomy/common/math/math.hpp"
#include "autonomy/map/costmap_2d/cost_values.hpp"
#include "autonomy/map/costmap_2d/costmap_2d_wrapper.hpp"

namespace autonomy {
namespace control {
namespace controller {
namespace tdmpc {
namespace mpc_opt {

namespace {

double SquaredYawError(double theta, double ref_theta) {
    const double err =
        ::autonomy::common::NormalizeAngleDifference(theta - ref_theta);
    return err * err;
}

}  // namespace

TdmpcCostWeights WeightsFromOptions(const proto::TdmpcControllerOptions& options) {
    TdmpcCostWeights w;
    w.q_contour = options.q_contour() > 0.0 ? options.q_contour() : 10.0;
    w.q_lag = options.q_lag() > 0.0 ? options.q_lag() : 10.0;
    w.q_yaw = options.q_yaw() > 0.0 ? options.q_yaw() : 5.0;
    w.q_velocity = options.q_velocity() > 0.0 ? options.q_velocity() : 1.0;
    w.q_contour_terminal = options.q_contour_terminal();
    w.q_lag_terminal = options.q_lag_terminal();
    w.q_yaw_terminal = options.q_yaw_terminal();
    w.r_v = options.r_v() > 0.0 ? options.r_v() : 0.1;
    w.r_omega = options.r_omega() > 0.0 ? options.r_omega() : 0.1;
    w.r_du = options.r_du() > 0.0 ? options.r_du() : 0.0;
    w.q_obstacle = options.q_obstacle() > 0.0 ? options.q_obstacle() : 50.0;
    w.obstacle_cost_threshold =
        options.obstacle_cost_threshold() > 0.0 ? options.obstacle_cost_threshold()
                                                : 200.0;
    return w;
}

double ContouringStageCost(const Pose2D& state, const tracking::PathSpline& spline,
                           double ref_arc_length,
                           const TdmpcCostWeights& weights) {
    double contour = 0.0;
    double lag = 0.0;
    spline.ContouringErrorsAtArcLength(state, ref_arc_length, &contour, &lag);
    const Pose2D ref = spline.PoseAtArcLength(ref_arc_length);
    return weights.q_contour * contour * contour + weights.q_lag * lag * lag +
           weights.q_yaw * SquaredYawError(state.theta, ref.theta);
}

double ContouringTerminalCost(const Pose2D& state,
                              const tracking::PathSpline& spline,
                              double ref_arc_length,
                              const TdmpcCostWeights& weights) {
    const double qc = weights.q_contour_terminal > 0.0 ? weights.q_contour_terminal
                                                       : weights.q_contour;
    const double ql =
        weights.q_lag_terminal > 0.0 ? weights.q_lag_terminal : weights.q_lag;
    const double qy =
        weights.q_yaw_terminal > 0.0 ? weights.q_yaw_terminal : weights.q_yaw;
    double contour = 0.0;
    double lag = 0.0;
    spline.ContouringErrorsAtArcLength(state, ref_arc_length, &contour, &lag);
    const Pose2D ref = spline.PoseAtArcLength(ref_arc_length);
    return qc * contour * contour + ql * lag * lag +
           qy * SquaredYawError(state.theta, ref.theta);
}

double ObstacleStageCost(const Pose2D& state,
                         const map::costmap_2d::Costmap2DWrapper* costmap_wrapper,
                         const TdmpcCostWeights& weights, bool enabled) {
    if (!enabled || !costmap_wrapper) {
        return 0.0;
    }
    // getCostmap() is non-const; obstacle query is read-only.
    auto* costmap =
        const_cast<map::costmap_2d::Costmap2DWrapper*>(costmap_wrapper)
            ->getCostmap();
    if (!costmap) {
        return 0.0;
    }
    unsigned int mx = 0;
    unsigned int my = 0;
    if (!costmap->worldToMap(state.x, state.y, mx, my)) {
        return weights.q_obstacle * 1.0;
    }
    const unsigned char cost = costmap->getCost(mx, my);
    if (cost >= map::costmap_2d::INSCRIBED_INFLATED_OBSTACLE) {
        return weights.q_obstacle * 10.0;
    }
    if (static_cast<double>(cost) > weights.obstacle_cost_threshold) {
        const double excess =
            (static_cast<double>(cost) - weights.obstacle_cost_threshold) / 100.0;
        return weights.q_obstacle * excess * excess;
    }
    return 0.0;
}

}  // namespace mpc_opt
}  // namespace tdmpc
}  // namespace controller
}  // namespace control
}  // namespace autonomy
