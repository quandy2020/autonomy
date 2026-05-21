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

#include <memory>
#include <vector>

#include "autonomy/control/controller/nmpc_controller/optimization/optimization_guard.hpp"
#include "autonomy/common/optimization/core/problem.hpp"
#include "autonomy/common/optimization/ipopt/ipopt_solver.hpp"
#include "autonomy/control/controller/nmpc_controller/models/kinematic_model.hpp"
#include "autonomy/control/controller/tdmpc_controller/models_alias.hpp"
#include "autonomy/control/controller/tdmpc_controller/optimization/tdmpc_cost.hpp"
#include "autonomy/control/controller/tdmpc_controller/optimization/tdmpc_control_variables.hpp"
#include "autonomy/control/controller/tdmpc_controller/optimization/tdmpc_tracking_cost.hpp"
#include "autonomy/control/controller/tdmpc_controller/topology/topology_manager.hpp"
#include "autonomy/control/controller/tdmpc_controller/tracking/contouring_reference.hpp"
#include "autonomy/control/controller/tdmpc_controller/tracking/path_spline.hpp"
#include "autonomy/control/proto/tdmpc_controller.pb.h"
#include "autonomy/map/costmap_2d/costmap_2d_wrapper.hpp"

namespace autonomy {
namespace control {
namespace controller {
namespace tdmpc {
namespace mpc_opt {

namespace ifopt = ::autonomy::common::optimization;

using models::BodyTwist;
using models::Pose2D;

/** T-MPC++ solver: topology selection + Ipopt (mpc_planner-style MPCC). */
class TdmpcSolver
{
public:
    explicit TdmpcSolver(const proto::TdmpcControllerOptions& options,
                         const map::costmap_2d::Costmap2DWrapper* costmap);

    bool Solve(const Pose2D& initial_state, const tracking::PathSpline& spline,
               BodyTwist& control_out);

    int selectedTopologyId() const { return selected_topology_id_; }
    bool usedFallback() const { return used_fallback_; }
    const models::KinematicModel& kinematicModel() const { return kinematic_model_; }
    const tracking::ContouringHorizon& lastHorizon() const { return last_horizon_; }

private:
    bool SelectTopology(const Pose2D& initial_state,
                        const tracking::PathSpline& spline,
                        tracking::ContouringHorizon* horizon_out);
    void ConfigureIpopt();
    void SyncInitialGuess();
    bool RunIpopt(BodyTwist& control_out);

    proto::TdmpcControllerOptions options_;
    models::KinematicModel kinematic_model_;
    topology::TopologyManager topology_;
    TdmpcCostWeights weights_;
    int horizon_{10};
    double dt_{0.1};
    int max_iterations_{50};
    double cost_tolerance_{1e-4};
    double ipopt_max_cpu_time_{0.08};
    bool use_fallback_on_failure_{true};
    bool enable_costmap_constraints_{true};

    const map::costmap_2d::Costmap2DWrapper* costmap_{nullptr};
    tracking::PathSpline active_spline_;
    tracking::ContouringReference contouring_ref_;

    std::shared_ptr<TdmpcControlVariables> control_variables_;
    std::shared_ptr<TdmpcTrackingCost> tracking_cost_;
    std::unique_ptr<ifopt::Problem> problem_;
    std::unique_ptr<ifopt::IpoptSolver> ipopt_;

    BodyTwist last_good_twist_;
    bool has_last_good_twist_{false};
    bool used_fallback_{false};
    int selected_topology_id_{0};
    tracking::ContouringHorizon last_horizon_;
};

}  // namespace mpc_opt
}  // namespace tdmpc
}  // namespace controller
}  // namespace control
}  // namespace autonomy
