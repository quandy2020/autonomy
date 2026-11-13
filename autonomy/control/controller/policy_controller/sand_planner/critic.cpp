/*
 * Copyright 2024 The OpenRobotic Beginner Authors (duyongquan)
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
 * @file critic.cpp
 * @brief ESDF generate-and-select critic using common/math clamped cubic B-splines.
 */

#include "autonomy/control/controller/policy_controller/sand_planner/critic.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <memory>

#include "autonomy/common/math/bspline.hpp"
#include "autonomy/control/controller/policy_controller/sand_planner/local_esdf.hpp"

namespace autonomy {
namespace control {
namespace controller {
namespace policy_controller {

namespace math = ::autonomy::common::math;

namespace {

Eigen::Vector3d RelativeGoal(const ObservationSnapshot& snapshot) {
    if (!snapshot.pose.has_value() || snapshot.path == nullptr ||
        snapshot.path->poses().empty()) {
        return Eigen::Vector3d::Zero();
    }
    const auto& pose = snapshot.pose->pose();
    const auto& goal =
        snapshot.path->poses(snapshot.path->poses_size() - 1).pose().position();
    const double yaw = std::atan2(
        2.0 * (pose.orientation().w() * pose.orientation().z() +
               pose.orientation().x() * pose.orientation().y()),
        1.0 - 2.0 * (pose.orientation().y() * pose.orientation().y() +
                     pose.orientation().z() * pose.orientation().z()));
    const double dx = goal.x() - pose.position().x();
    const double dy = goal.y() - pose.position().y();
    const double dz = goal.z() - pose.position().z();
    const double c = std::cos(-yaw);
    const double s = std::sin(-yaw);
    return {c * dx - s * dy, s * dx + c * dy, dz};
}

std::vector<Eigen::Vector3d> SubsampleWaypoints(
    const std::vector<Eigen::Vector3d>& waypoints, int sample_count) {
    if (waypoints.empty() || sample_count <= 0 ||
        static_cast<int>(waypoints.size()) <= sample_count) {
        return waypoints;
    }
    std::vector<Eigen::Vector3d> samples(
        static_cast<std::size_t>(sample_count));
    const int last = static_cast<int>(waypoints.size()) - 1;
    for (int i = 0; i < sample_count; ++i) {
        const double t = static_cast<double>(i) /
                         static_cast<double>(sample_count - 1);
        const int index = static_cast<int>(std::round(t * last));
        samples[static_cast<std::size_t>(i)] =
            waypoints[static_cast<std::size_t>(index)];
    }
    return samples;
}

}  // namespace

std::vector<Eigen::Vector3d> ControlPointsFromFlat(
    const std::vector<float>& raw, int control_point_count) {
    const int count = std::max(4, control_point_count);
    std::vector<Eigen::Vector3d> points(
        static_cast<std::size_t>(count), Eigen::Vector3d::Zero());
    const std::size_t need = static_cast<std::size_t>(count) * 3;
    for (int i = 0; i < count; ++i) {
        const std::size_t base = static_cast<std::size_t>(i) * 3;
        if (base + 2 >= raw.size() || base + 2 >= need) {
            break;
        }
        points[static_cast<std::size_t>(i)] = Eigen::Vector3d(
            raw[base], raw[base + 1], raw[base + 2]);
    }
    points.front() = Eigen::Vector3d::Zero();
    return points;
}

double EvaluateSandPlannerCost(const std::vector<Eigen::Vector3d>& waypoints,
                               const Eigen::Vector3d& goal,
                               const proto::SandPlanner& options,
                               const map::esdf::EsdfMap* esdf) {
    if (waypoints.empty()) {
        return std::numeric_limits<double>::infinity();
    }
    double length = 0.0;
    for (std::size_t i = 1; i < waypoints.size(); ++i) {
        length += (waypoints[i] - waypoints[i - 1]).norm();
    }
    const double goal_error = (waypoints.back() - goal).norm();
    const double goal_cost = goal_error * goal_error;

    double safety = 0.0;
    double weight_sum = 0.0;
    if (esdf != nullptr) {
        const std::vector<Eigen::Vector3d> samples = SubsampleWaypoints(
            waypoints, options.clearance_samples());
        const double gamma = options.discount();
        const double d_safe = options.safety_margin();
        const double d_pref = options.preferred_clearance();
        double weight = 1.0;
        for (std::size_t i = 0; i < samples.size(); ++i) {
            const double clearance = QueryLocalClearance(*esdf, samples[i]);
            if (std::isfinite(clearance)) {
                const double under = std::max(0.0, d_safe - clearance);
                const double over = std::max(0.0, clearance - d_pref);
                safety += weight * (under * under + 0.1 * over * over);
                weight_sum += weight;
            }
            weight *= gamma;
        }
    }
    const double safety_cost =
        weight_sum > 1e-9 ? safety / weight_sum : 0.0;
    return options.lambda_safety() * safety_cost +
           options.lambda_length() * length +
           options.lambda_goal() * goal_cost;
}

bool SelectSandPlannerTwist(
    const std::vector<std::vector<float>>& candidates,
    const ObservationSnapshot& snapshot, const proto::SandPlanner& options,
    automsgs::msgs::geometry_msgs::Twist* twist,
    Eigen::Vector3d* selected_heading, std::string* error) {
    if (twist == nullptr) {
        return false;
    }
    if (candidates.empty()) {
        if (error != nullptr) {
            *error = "SanD-Planner produced no candidates";
        }
        return false;
    }
    std::string esdf_error;
    const std::unique_ptr<map::esdf::EsdfMap> esdf =
        BuildLocalEsdfFromDepth(snapshot, options, &esdf_error);
    const Eigen::Vector3d goal = RelativeGoal(snapshot);
    const int control_points = std::max(4, options.control_points());
    const int waypoints = std::max(2, options.waypoint_count());
    double best_cost = std::numeric_limits<double>::infinity();
    std::vector<Eigen::Vector3d> best_controls;
    for (const auto& raw : candidates) {
        std::vector<Eigen::Vector3d> controls =
            ControlPointsFromFlat(raw, control_points);
        const math::ClampedCubicBSpline<Eigen::Vector3d> curve(controls);
        const std::vector<Eigen::Vector3d> dense =
            curve.Sample(std::max(waypoints, 16));
        const std::vector<Eigen::Vector3d> samples =
            math::ResamplePolylineArcLength(dense, waypoints);
        const double cost =
            EvaluateSandPlannerCost(samples, goal, options, esdf.get());
        if (cost < best_cost) {
            best_cost = cost;
            best_controls = std::move(controls);
        }
    }
    if (best_controls.size() < 2) {
        if (error != nullptr) {
            *error = "SanD-Planner candidate has too few control points";
        }
        return false;
    }
    Eigen::Vector3d heading = best_controls[1];
    const double norm = heading.norm();
    if (norm < 1e-6) {
        heading = Eigen::Vector3d(1.0, 0.0, 0.0);
    } else {
        heading /= norm;
    }
    if (selected_heading != nullptr) {
        *selected_heading = heading;
    }
    const double speed = options.execute_speed();
    twist->Clear();
    twist->mutable_linear()->set_x(speed * heading.x());
    twist->mutable_linear()->set_y(speed * heading.y());
    twist->mutable_linear()->set_z(speed * heading.z());
    twist->mutable_angular()->set_z(std::atan2(heading.y(), heading.x()));
    return true;
}

}  // namespace policy_controller
}  // namespace controller
}  // namespace control
}  // namespace autonomy
