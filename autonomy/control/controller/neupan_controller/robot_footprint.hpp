/*
 * Copyright 2026 The Openbot Authors (duyongquan)
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
 * @file robot_footprint.hpp
 * @brief Convex robot footprint and NeuPAN halfspace form A p <= b.
 */

#pragma once

#include <string>

#include <automsgs/msgs/geometry_msgs/polygon.pb.h>
#include <Eigen/Core>

namespace autonomy {
namespace control {
namespace controller {
namespace neupan_controller {

/**
 * @brief Convex robot footprint in the base frame with halfspace constraints.
 *
 * Vertices are stored as an @c automsgs::msgs::geometry_msgs::Polygon (CCW).
 * Halfspaces match NeuPAN @c gen_inequal_from_vertex (unnormalized G/h) and are
 * consumed by @ref DistanceEncoder.
 */
struct RobotFootprint {
    /** @brief Kinematics label: "differential" or "ackermann". */
    std::string kinematics = "differential";
    /** @brief Rectangle length [m] when built via @ref CreateRectangle. */
    double length = 0.8;
    /** @brief Rectangle width [m] when built via @ref CreateRectangle. */
    double width = 0.6;
    /** @brief Wheelbase [m] (Ackermann rear-axle offset). */
    double wheelbase = 0.5;
    /** @brief CCW polygon vertices in the robot base frame. */
    automsgs::msgs::geometry_msgs::Polygon polygon;

    /** @brief Halfspace matrix A, shape [edge_count, 2], with A p <= b. */
    Eigen::MatrixXd halfspace_matrix;
    /** @brief Halfspace offsets b, shape [edge_count]. */
    Eigen::VectorXd halfspace_offset;

    /**
     * @brief Rebuild @ref halfspace_matrix / @ref halfspace_offset from @ref
     * polygon.
     *
     * Requires at least three vertices; otherwise clears the halfspaces.
     */
    void ComputeHalfspaceConstraints();

    /**
     * @brief Number of halfspace edges (equals polygon side count when valid).
     * @return Edge count, or 0 if constraints are empty.
     */
    int GetEdgeCount() const {
        return static_cast<int>(halfspace_offset.size());
    }

    /**
     * @brief Build an axis-aligned rectangular footprint and its halfspaces.
     * @param[in] kinematics "differential" (centered) or "ackermann"
     * (rear-biased).
     * @param[in] length Footprint length along x [m].
     * @param[in] width Footprint width along y [m].
     * @param[in] wheelbase Used only for Ackermann vertex placement [m].
     * @return Populated footprint with halfspaces computed.
     */
    static RobotFootprint CreateRectangle(const std::string& kinematics,
                                          double length, double width,
                                          double wheelbase);

    /**
     * @brief Build a footprint from an arbitrary convex CCW polygon.
     * @param[in] kinematics Kinematics label stored on the footprint.
     * @param[in] polygon CCW vertices in the robot base frame.
     * @param[in] wheelbase Wheelbase [m] (metadata; not used for vertex
     * layout).
     * @return Populated footprint with halfspaces computed.
     */
    static RobotFootprint CreateFromPolygon(
        const std::string& kinematics,
        const automsgs::msgs::geometry_msgs::Polygon& polygon,
        double wheelbase);
};

}  // namespace neupan_controller
}  // namespace controller
}  // namespace control
}  // namespace autonomy
