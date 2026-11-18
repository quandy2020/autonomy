/*
 * Copyright 2026 The Openbot Authors
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

/**
 * @file smooth_control_law.hpp
 * @brief Park & Kuipers control law (port of nav2_graceful_controller).
 */

#pragma once

#include <algorithm>
#include <cmath>

#include <automsgs/msgs/geometry_msgs/pose.pb.h>
#include <automsgs/msgs/geometry_msgs/twist.pb.h>
#include "autonomy/task/charging/dock/geometry.hpp"

namespace autonomy {
namespace task {
namespace charging {

struct EgoPolar {
    double r{0.0};
    double phi{0.0};
    double delta{0.0};
};

inline EgoPolar MakeEgoPolar(const ::automsgs::msgs::geometry_msgs::Pose& target,
                             const ::automsgs::msgs::geometry_msgs::Pose& current,
                             bool backward) {
    const double dX = target.position().x() - current.position().x();
    const double dY = target.position().y() - current.position().y();
    const double line_of_sight =
        backward ? (std::atan2(-dY, dX) + M_PI) : std::atan2(-dY, dX);
    EgoPolar c;
    c.r = std::hypot(dX, dY);
    c.phi = geom::NormalizeAngle(geom::Yaw(target) + line_of_sight);
    c.delta = geom::NormalizeAngle(geom::Yaw(current) + line_of_sight);
    return c;
}

/**
 * @class autonomy::task::charging::SmoothControlLaw
 * @brief Curvature-bounded spiral toward a robot-centric target pose.
 */
class SmoothControlLaw {
public:
    SmoothControlLaw(double k_phi, double k_delta, double beta, double lambda,
                     double slowdown_radius, double deceleration_max,
                     double v_linear_min, double v_linear_max,
                     double v_angular_max)
        : k_phi_(k_phi),
          k_delta_(k_delta),
          beta_(beta),
          lambda_(lambda),
          slowdown_radius_(slowdown_radius),
          deceleration_max_(deceleration_max),
          v_linear_min_(v_linear_min),
          v_linear_max_(v_linear_max),
          v_angular_max_(v_angular_max) {}

    ::automsgs::msgs::geometry_msgs::Twist Calculate(
        const ::automsgs::msgs::geometry_msgs::Pose& target, bool backward) const {
        ::automsgs::msgs::geometry_msgs::Pose origin;
        origin.mutable_orientation()->set_w(1.0);
        return Calculate(target, origin, backward);
    }

    ::automsgs::msgs::geometry_msgs::Twist Calculate(
        const ::automsgs::msgs::geometry_msgs::Pose& target,
        const ::automsgs::msgs::geometry_msgs::Pose& current, bool backward) const {
        const auto ego = MakeEgoPolar(target, current, backward);
        const double r = std::max(ego.r, 1e-3);
        const double prop = k_delta_ * (ego.delta - std::atan(-k_phi_ * ego.phi));
        const double feedback =
            (1.0 + (k_phi_ / (1.0 + std::pow(k_phi_ * ego.phi, 2)))) *
            std::sin(ego.delta);
        double curvature = -1.0 / r * (prop + feedback);
        if (backward) {
            curvature = -curvature;
        }
        double v = v_linear_max_ / (1.0 + beta_ * std::pow(std::fabs(curvature), lambda_));
        v = std::min(v_linear_max_ * (ego.r / std::max(slowdown_radius_, 1e-3)), v);
        v = std::min(std::sqrt(std::max(0.0, 2.0 * ego.r * deceleration_max_)), v);
        v = std::clamp(v, v_linear_min_, v_linear_max_);
        if (backward) {
            v = -v;
        }
        double w = curvature * v;
        const double w_bound = std::clamp(w, -v_angular_max_, v_angular_max_);
        if (curvature != 0.0) {
            v = w_bound / curvature;
        }
        ::automsgs::msgs::geometry_msgs::Twist cmd;
        cmd.mutable_linear()->set_x(v);
        cmd.mutable_angular()->set_z(w_bound);
        return cmd;
    }

    ::automsgs::msgs::geometry_msgs::Pose NextPose(
        double dt, const ::automsgs::msgs::geometry_msgs::Pose& target,
        const ::automsgs::msgs::geometry_msgs::Pose& current, bool backward) const {
        const auto vel = Calculate(target, current, backward);
        const double yaw = geom::Yaw(current);
        ::automsgs::msgs::geometry_msgs::Pose next;
        next.mutable_position()->set_x(current.position().x() +
                                       vel.linear().x() * dt * std::cos(yaw));
        next.mutable_position()->set_y(current.position().y() +
                                       vel.linear().x() * dt * std::sin(yaw));
        geom::SetYaw(&next, geom::NormalizeAngle(yaw + vel.angular().z() * dt));
        return next;
    }

private:
    double k_phi_{3.0};
    double k_delta_{2.0};
    double beta_{0.4};
    double lambda_{2.0};
    double slowdown_radius_{0.25};
    double deceleration_max_{0.5};
    double v_linear_min_{0.1};
    double v_linear_max_{0.25};
    double v_angular_max_{0.75};
};

}  // namespace charging
}  // namespace task
}  // namespace autonomy
