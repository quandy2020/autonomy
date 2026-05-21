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
#include <vector>

#include "autonomy/control/controller/nmpc_controller/optimization/mpc_cost_term_base.hpp"
#include "autonomy/control/controller/nmpc_controller/models/kinematic_model.hpp"
#include "autonomy/control/controller/nmpc_controller/models/kinematic_pose.hpp"
#include "autonomy/control/controller/nmpc_controller/optimization/mpc_cost.hpp"
#include "autonomy/control/controller/nmpc_controller/optimization/mpc_control_variables.hpp"

namespace autonomy {
namespace control {
namespace controller {
namespace nmpc {
namespace mpc_opt {

/** Tracking cost for the NMPC control sequence (single-shooting rollout). */
class MpcTrackingCost : public MpcCostTermBase
{
public:
    MpcTrackingCost();

    void SetContext(const models::Pose2D& initial_state,
                    const std::vector<models::Pose2D>& references, double dt,
                    const models::KinematicModel& kinematic_model,
                    const MpcCostWeights& weights);

    double GetCost() const override;

    void FillJacobianBlock(
        std::string var_set,
        ::autonomy::common::optimization::Component::Jacobian& jac) const override;

private:
    models::Pose2D initial_state_;
    std::vector<models::Pose2D> references_;
    double dt_{0.1};
    std::optional<models::KinematicModel> kinematic_model_;
    MpcCostWeights weights_;
};

}  // namespace mpc_opt
}  // namespace nmpc
}  // namespace controller
}  // namespace control
}  // namespace autonomy
