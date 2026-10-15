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

#include "autonomy/control/controller/neupan_controller/robot_footprint.hpp"

namespace autonomy {
namespace control {
namespace controller {
namespace neupan_controller {
namespace {

void AppendPolygonVertex(automsgs::msgs::geometry_msgs::Polygon* polygon,
                         double x, double y) {
    auto* point = polygon->add_points();
    point->set_x(static_cast<float>(x));
    point->set_y(static_cast<float>(y));
    point->set_z(0.0f);
}

automsgs::msgs::geometry_msgs::Polygon CreateRectanglePolygon(
    const std::string& kinematics, double length, double width,
    double wheelbase) {
    automsgs::msgs::geometry_msgs::Polygon polygon;
    const double half_width = 0.5 * width;
    if (kinematics == "ackermann") {
        const double rear = 0.5 * (length - wheelbase);
        const double front = length - rear;
        AppendPolygonVertex(&polygon, -rear, -half_width);
        AppendPolygonVertex(&polygon, front, -half_width);
        AppendPolygonVertex(&polygon, front, half_width);
        AppendPolygonVertex(&polygon, -rear, half_width);
    } else {
        const double half_length = 0.5 * length;
        AppendPolygonVertex(&polygon, -half_length, -half_width);
        AppendPolygonVertex(&polygon, half_length, -half_width);
        AppendPolygonVertex(&polygon, half_length, half_width);
        AppendPolygonVertex(&polygon, -half_length, half_width);
    }
    return polygon;
}

}  // namespace

void RobotFootprint::ComputeHalfspaceConstraints() {
    const int n = polygon.points_size();
    if (n < 3) {
        halfspace_matrix.resize(0, 2);
        halfspace_offset.resize(0);
        return;
    }
    halfspace_matrix.resize(n, 2);
    halfspace_offset.resize(n);
    for (int i = 0; i < n; ++i) {
        const auto& p0 = polygon.points(i);
        const auto& p1 = polygon.points((i + 1) % n);
        const double ex =
            static_cast<double>(p1.x()) - static_cast<double>(p0.x());
        const double ey =
            static_cast<double>(p1.y()) - static_cast<double>(p0.y());
        // NeuPAN gen_inequal_from_vertex: n = (ey, -ex), h = n · p0
        // (unnormalized).
        halfspace_matrix(i, 0) = ey;
        halfspace_matrix(i, 1) = -ex;
        halfspace_offset(i) =
            ey * static_cast<double>(p0.x()) - ex * static_cast<double>(p0.y());
    }
}

RobotFootprint RobotFootprint::CreateRectangle(const std::string& kinematics,
                                               double length, double width,
                                               double wheelbase) {
    RobotFootprint footprint;
    footprint.kinematics = kinematics;
    footprint.length = length;
    footprint.width = width;
    footprint.wheelbase = wheelbase;
    footprint.polygon =
        CreateRectanglePolygon(kinematics, length, width, wheelbase);
    footprint.ComputeHalfspaceConstraints();
    return footprint;
}

RobotFootprint RobotFootprint::CreateFromPolygon(
    const std::string& kinematics,
    const automsgs::msgs::geometry_msgs::Polygon& polygon, double wheelbase) {
    RobotFootprint footprint;
    footprint.kinematics = kinematics;
    footprint.wheelbase = wheelbase;
    footprint.polygon = polygon;
    footprint.ComputeHalfspaceConstraints();
    return footprint;
}

}  // namespace neupan_controller
}  // namespace controller
}  // namespace control
}  // namespace autonomy
