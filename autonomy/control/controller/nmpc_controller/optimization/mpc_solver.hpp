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
#include "autonomy/control/controller/nmpc_controller/models/kinematic_pose.hpp"
#include "autonomy/control/controller/nmpc_controller/optimization/mpc_control_variables.hpp"
#include "autonomy/control/controller/nmpc_controller/optimization/mpc_cost.hpp"
#include "autonomy/control/controller/nmpc_controller/optimization/mpc_tracking_cost.hpp"
#include "autonomy/control/proto/nmpc_controller.pb.h"

namespace autonomy {
namespace control {
namespace controller {
namespace nmpc {
namespace mpc_opt {

namespace ifopt = ::autonomy::common::optimization;

using models::BodyTwist;
using models::KinematicModel;
using models::Pose2D;

/**
 * @brief NMPC solver using autonomy::common::optimization (Ipopt + ifopt-style
 *        Problem formulation).
 */
class MpcSolver
{
public:
    explicit MpcSolver(const proto::NmpcControllerOptions& options);

    bool Solve(const Pose2D& initial_state,
               const std::vector<Pose2D>& references, BodyTwist& control_out);

    const KinematicModel& kinematicModel() const { return kinematic_model_; }

    const std::vector<models::Control2D>& lastControlSequence() const;

    const std::vector<Pose2D>& lastPredictedStates() const {
        return predicted_states_;
    }

    bool usedFallback() const { return used_fallback_; }

private:
    void ConfigureIpopt();
    void RolloutPredictedStates(const Pose2D& initial_state,
                                const std::vector<Pose2D>& references);
    void SyncInitialGuess();
    bool StoreSolution(BodyTwist& control_out);

    models::KinematicModel kinematic_model_;
    int horizon_{10};
    double dt_{0.1};
    MpcCostWeights weights_;
    double cost_tolerance_{1e-4};
    int max_iterations_{30};
    double ipopt_max_cpu_time_{0.05};
    bool use_fallback_on_failure_{true};

    std::shared_ptr<MpcControlVariables> control_variables_;
    std::shared_ptr<MpcTrackingCost> tracking_cost_;
    std::unique_ptr<ifopt::Problem> problem_;
    std::unique_ptr<ifopt::IpoptSolver> ipopt_;

    std::vector<Pose2D> predicted_states_;
    BodyTwist last_good_twist_;
    bool has_last_good_twist_{false};
    bool used_fallback_{false};
};

}  // namespace mpc_opt
}  // namespace nmpc
}  // namespace controller
}  // namespace control
}  // namespace autonomy
