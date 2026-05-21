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

#include <optional>

#include "autonomy/control/controller/tdmpc_controller/optimization/tdmpc_cost_term_base.hpp"
#include "autonomy/control/controller/tdmpc_controller/models_alias.hpp"
#include "autonomy/control/controller/nmpc_controller/models/kinematic_model.hpp"
#include "autonomy/control/controller/tdmpc_controller/optimization/tdmpc_cost.hpp"
#include "autonomy/control/controller/tdmpc_controller/optimization/tdmpc_control_variables.hpp"
#include "autonomy/control/controller/tdmpc_controller/tracking/contouring_reference.hpp"
#include "autonomy/control/controller/tdmpc_controller/tracking/path_spline.hpp"
#include "autonomy/map/costmap_2d/costmap_2d_wrapper.hpp"

namespace autonomy {
namespace control {
namespace controller {
namespace tdmpc {
namespace mpc_opt {

double EvaluateRolloutCost(
    const models::Pose2D& initial_state, const tracking::PathSpline& spline,
    const tracking::ContouringHorizon& horizon,
    const std::vector<models::Control2D>& controls, double dt,
    const models::KinematicModel& kinematic_model, const TdmpcCostWeights& weights,
    const map::costmap_2d::Costmap2DWrapper* costmap, bool enable_obstacle);

class TdmpcTrackingCost : public TdmpcCostTermBase
{
public:
    TdmpcTrackingCost();

    void SetContext(const models::Pose2D& initial_state,
                    const tracking::PathSpline& spline,
                    const tracking::ContouringHorizon& horizon,
                    double dt, const models::KinematicModel& kinematic_model,
                    const TdmpcCostWeights& weights,
                    const map::costmap_2d::Costmap2DWrapper* costmap,
                    bool enable_obstacle_cost);

    double GetCost() const override;

    void FillJacobianBlock(
        std::string var_set,
        ::autonomy::common::optimization::Component::Jacobian& jac) const override;

private:
    models::Pose2D initial_state_;
    tracking::PathSpline spline_;
    tracking::ContouringHorizon horizon_;
    double dt_{0.1};
    std::optional<models::KinematicModel> kinematic_model_;
    TdmpcCostWeights weights_;
    const map::costmap_2d::Costmap2DWrapper* costmap_{nullptr};
    bool enable_obstacle_cost_{false};
};

}  // namespace mpc_opt
}  // namespace tdmpc
}  // namespace controller
}  // namespace control
}  // namespace autonomy
