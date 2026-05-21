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
#include "autonomy/control/controller/tdmpc_controller/models_alias.hpp"

namespace autonomy {
namespace control {
namespace controller {
namespace tdmpc {
namespace tracking {

using models::Pose2D;

/** Arc-length parameterized 2D path (mpc_planner Contouring spline abstraction). */
class PathSpline
{
public:
    bool BuildFromPath(const commsgs::planning_msgs::Path& path);

    bool empty() const { return cumulative_.empty(); }
    double length() const;

    size_t FindClosestIndex(const Pose2D& query) const;
    double ArcLengthAtIndex(size_t index) const;

    Pose2D PoseAtArcLength(double s) const;

    /** Signed contour (along-track) and lag (cross-track) errors [m]. */
    void ContouringErrors(const Pose2D& state, double* contour, double* lag) const;

    void ContouringErrorsAtArcLength(const Pose2D& state, double s,
                                     double* contour, double* lag) const;

    /** Normal vector (nx, ny) at arc length s (points to left of path). */
    void NormalAtArcLength(double s, double* nx, double* ny) const;

private:
    commsgs::planning_msgs::Path path_;
    std::vector<double> cumulative_;
};

}  // namespace tracking
}  // namespace tdmpc
}  // namespace controller
}  // namespace control
}  // namespace autonomy
