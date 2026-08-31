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

/**
 * @file path_reference.hpp
 * @brief Arc-length path sampling for NMPC horizon references
 */

#pragma once

#include <cmath>
#include <vector>

#include <automsgs/msgs/geometry_msgs/pose_stamped.pb.h>
#include <automsgs/msgs/nav_msgs/path.pb.h>

namespace autonomy {
namespace control {
namespace controller {
namespace nmpc_controller {

/**
 * @class nmpc_controller::PathReference
 * @brief Samples a local plan into per-horizon reference poses
 */
class PathReference {
 public:
    // Planar pose used for NMPC references
    struct Pose2D {
        // Position x in the costmap frame [m]
        double x = 0.0;
        // Position y in the costmap frame [m]
        double y = 0.0;
        // Heading yaw in the costmap frame [rad]
        double yaw = 0.0;
    };

    // Closest-point projection result on the path
    struct ClosestPoint {
        // Arc length of the projected point [m]
        double arc_length = 0.0;
        // Euclidean distance from the query point to the path [m]
        double distance = 0.0;
    };

    /**
     * @brief Load a new local plan and rebuild cumulative arc length
     * @param plan Path in the costmap frame
     */
    void SetPlan(const automsgs::msgs::nav_msgs::Path& plan);

    /**
     * @brief Whether the reference path is empty
     */
    bool empty() const { return poses_.empty(); }

    /**
     * @brief Sample a pose along the path by arc length (linear position, slerp yaw)
     * @param start_s Starting arc length [m]
     * @param step Arc-length step between horizon samples [m]
     * @param count Horizon step index
     */
    Pose2D SampleAlongPath(double start_s, double step, int count) const;

    /**
     * @brief Terminal planar pose of the path
     */
    Pose2D Goal() const;

    /**
     * @brief Terminal full pose of the path (for goal checking)
     */
    automsgs::msgs::geometry_msgs::Pose GoalPose() const { return goal_pose_; }

    /**
     * @brief Project a point onto the path forward from a search window
     * @param x Query x [m]
     * @param y Query y [m]
     * @param min_arc_length Minimum arc length (monotonic progress)
     * @param search_behind Look-behind distance along the path [m]
     */
    ClosestPoint ClosestPointOnPath(double x, double y, double min_arc_length,
                                    double search_behind) const;

    /**
     * @brief Total path length [m]
     */
    double PathLength() const { return path_length_; }

 private:
    static double YawFromPose(const automsgs::msgs::geometry_msgs::Pose& pose);

    // Planar poses along the local path
    std::vector<Pose2D> poses_;
    // Cumulative arc length at each poses_ entry [m]
    std::vector<double> cumulative_s_;
    // Full terminal pose (position + orientation) for goal checking
    automsgs::msgs::geometry_msgs::Pose goal_pose_;
    // Total arc length of the loaded local path [m]
    double path_length_ = 0.0;
};

}  // namespace nmpc_controller
}  // namespace controller
}  // namespace control
}  // namespace autonomy
