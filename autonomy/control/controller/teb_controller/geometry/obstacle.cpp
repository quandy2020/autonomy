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

#include "autonomy/control/controller/teb_controller/geometry/obstacle.hpp"

namespace autonomy {
namespace control {
namespace teb_controller {

Obstacle::Obstacle() : dynamic_(false), centroid_velocity_{} {}

void Obstacle::PredictCentroidConstantVelocity(double t, Point& position) const {
    position = PredictPosition(GetCentroid(), centroid_velocity_, t);
}

bool Obstacle::IsDynamic() const {
    return dynamic_;
}

void Obstacle::SetCentroidVelocity(const Twist2D& vel) {
    centroid_velocity_ = vel;
    dynamic_ = true;
}

void Obstacle::SetCentroidVelocity(
    const autonomy::commsgs::geometry_msgs::TwistWithCovariance& velocity,
    const autonomy::commsgs::geometry_msgs::Quaternion& /*orientation*/) {
    Twist2D vel{};
    vel.x = velocity.twist.linear.x;
    vel.y = velocity.twist.linear.y;
    vel.theta = 0.0f;

    constexpr double kMinDynamicVelocity = 0.001;
    if (std::hypot(vel.x, vel.y) < kMinDynamicVelocity) {
        return;
    }

    SetCentroidVelocity(vel);
}

void Obstacle::SetCentroidVelocity(
    const autonomy::commsgs::geometry_msgs::TwistWithCovariance& velocity,
    const autonomy::commsgs::geometry_msgs::QuaternionStamped& orientation) {
    SetCentroidVelocity(velocity, orientation.quaternion);
}

const Twist2D& Obstacle::GetCentroidVelocity() const {
    return centroid_velocity_;
}

void Obstacle::ToTwistWithCovarianceMsg(
    autonomy::commsgs::geometry_msgs::TwistWithCovariance& twistWithCovariance) {
    if (dynamic_) {
        twistWithCovariance.twist.linear.x = centroid_velocity_.x;
        twistWithCovariance.twist.linear.y = centroid_velocity_.y;
    } else {
        twistWithCovariance.twist.linear.x = 0;
        twistWithCovariance.twist.linear.y = 0;
    }
}

}  // namespace teb_controller
}  // namespace control
}  // namespace autonomy
