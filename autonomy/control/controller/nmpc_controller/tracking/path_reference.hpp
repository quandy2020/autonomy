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

#include "autonomy/commsgs/planning_msgs.hpp"
#include "autonomy/control/controller/nmpc_controller/models/kinematic_pose.hpp"

namespace autonomy {
namespace control {
namespace controller {
namespace nmpc {
namespace tracking {

using models::Pose2D;

/**
 * @brief Builds horizon reference states by marching along the path.
 */
class PathReference
{
public:
    /**
     * @param horizon Number of prediction steps (N); produces N+1 references.
     * @param dt Time step between stages (s).
     * @param reference_velocity Nominal speed along path (m/s).
     */
    PathReference(int horizon, double dt, double reference_velocity,
                  double slowdown_radius = 0.0);

    bool BuildFromPath(const commsgs::planning_msgs::Path& path,
                       const Pose2D& current_state);

    /** Nominal speed used for the latest reference horizon (m/s). */
    double effectiveReferenceVelocity() const {
        return effective_reference_velocity_;
    }

    const std::vector<Pose2D>& references() const {
        return references_;
    }

private:
    static Pose2D InterpolateAlongPath(const commsgs::planning_msgs::Path& path,
                                       const std::vector<double>& cumulative,
                                       double arc_length);

    int horizon_{0};
    double dt_{0.1};
    double reference_velocity_{0.3};
    double slowdown_radius_{0.0};
    double effective_reference_velocity_{0.3};
    std::vector<Pose2D> references_;
};

}  // namespace tracking
}  // namespace nmpc
}  // namespace controller
}  // namespace control
}  // namespace autonomy
