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

#include "autonomy/control/controller/tdmpc_controller/tracking/contouring_reference.hpp"

#include <algorithm>

namespace autonomy {
namespace control {
namespace controller {
namespace tdmpc {
namespace tracking {

ContouringReference::ContouringReference(
    int horizon, double dt, const proto::TdmpcControllerOptions& options)
    : horizon_steps_(horizon > 0 ? horizon : 1),
      dt_(dt > 0.0 ? dt : 0.1),
      reference_velocity_(options.reference_velocity() > 0.0
                              ? options.reference_velocity()
                              : 0.3),
      slowdown_radius_(options.slowdown_radius() > 0.0
                           ? options.slowdown_radius()
                           : 0.0) {}

bool ContouringReference::Build(const PathSpline& spline,
                                const Pose2D& current_state,
                                double lateral_offset) {
    contouring_horizon_.arc_lengths.clear();
    contouring_horizon_.poses.clear();
    contouring_horizon_.arc_lengths.resize(static_cast<size_t>(horizon_steps_ + 1));
    contouring_horizon_.poses.resize(static_cast<size_t>(horizon_steps_ + 1));

    if (spline.empty()) {
        contouring_horizon_.reference_velocity = 0.0;
        return false;
    }

    const size_t closest = spline.FindClosestIndex(current_state);
    const double arc_origin = spline.ArcLengthAtIndex(closest);
    const double dist_to_goal = std::max(spline.length() - arc_origin, 0.0);

    double ref_v = reference_velocity_;
    if (slowdown_radius_ > 0.0 && dist_to_goal < slowdown_radius_) {
        ref_v = reference_velocity_ * (dist_to_goal / slowdown_radius_);
        ref_v = std::max(ref_v, reference_velocity_ * 0.05);
    }
    contouring_horizon_.reference_velocity = ref_v;

    const double step = ref_v * dt_;
    for (int k = 0; k <= horizon_steps_; ++k) {
        const double s = arc_origin + step * static_cast<double>(k);
        contouring_horizon_.arc_lengths[static_cast<size_t>(k)] = s;
        Pose2D ref = spline.PoseAtArcLength(s);
        if (std::abs(lateral_offset) > 1e-6) {
            double nx = 0.0;
            double ny = 0.0;
            spline.NormalAtArcLength(s, &nx, &ny);
            ref.x += lateral_offset * nx;
            ref.y += lateral_offset * ny;
        }
        contouring_horizon_.poses[static_cast<size_t>(k)] = ref;
    }
    return true;
}

}  // namespace tracking
}  // namespace tdmpc
}  // namespace controller
}  // namespace control
}  // namespace autonomy
