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

#include <vector>

#include "autonomy/control/controller/nmpc_controller/models/kinematic_pose.hpp"
#include "autonomy/control/controller/tdmpc_controller/models_alias.hpp"
#include "autonomy/control/controller/tdmpc_controller/tracking/path_spline.hpp"
#include "autonomy/control/proto/tdmpc_controller.pb.h"

namespace autonomy {
namespace control {
namespace controller {
namespace tdmpc {
namespace tracking {

using models::Pose2D;

/** Horizon arc-length references for MPCC (per topology lateral offset). */
struct ContouringHorizon {
    std::vector<double> arc_lengths;
    std::vector<Pose2D> poses;
    double reference_velocity{0.3};
};

class ContouringReference
{
public:
    ContouringReference(int horizon, double dt,
                        const proto::TdmpcControllerOptions& options);

    bool Build(const PathSpline& spline, const Pose2D& current_state,
               double lateral_offset);

    const ContouringHorizon& contouringHorizon() const {
        return contouring_horizon_;
    }

private:
    int horizon_steps_{0};
    double dt_{0.1};
    double reference_velocity_{0.3};
    double slowdown_radius_{0.0};
    ContouringHorizon contouring_horizon_;
};

}  // namespace tracking
}  // namespace tdmpc
}  // namespace controller
}  // namespace control
}  // namespace autonomy
