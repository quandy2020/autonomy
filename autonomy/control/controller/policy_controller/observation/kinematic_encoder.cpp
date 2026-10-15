/*
 * Copyright 2025 The Openbot Authors (duyongquan)
 */

#include "autonomy/control/controller/policy_controller/observation/kinematic_encoder.hpp"

#include <algorithm>
#include <cmath>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

namespace autonomy {
namespace control {
namespace controller {
namespace policy_controller {
namespace {

double YawFromQuaternion(
    const automsgs::msgs::geometry_msgs::Quaternion& quaternion) {
    const double siny_cosp =
        2.0 * (quaternion.w() * quaternion.z() + quaternion.x() * quaternion.y());
    const double cosy_cosp =
        1.0 - 2.0 * (quaternion.y() * quaternion.y() +
                     quaternion.z() * quaternion.z());
    return std::atan2(siny_cosp, cosy_cosp);
}

std::size_t ExpectedElements(const proto::ObservationInput& input,
                             std::size_t fallback) {
    if (input.shape_size() == 0) {
        return fallback;
    }
    std::size_t count = 1;
    bool any = false;
    for (int i = 0; i < input.shape_size(); ++i) {
        if (input.shape(i) > 0) {
            count *= static_cast<std::size_t>(input.shape(i));
            any = true;
        }
    }
    return any ? count : fallback;
}

}  // namespace

bool EncodeKinematic(const ObservationSnapshot& snapshot,
                     const proto::ObservationInput& input,
                     std::vector<float>* output, std::string* error,
                     bool relative_to_path_goal) {
    if (output == nullptr) {
        return false;
    }
    if (!snapshot.pose.has_value() || !snapshot.velocity.has_value()) {
        if (error != nullptr) {
            *error = "Kinematic modality requires pose and velocity";
        }
        return false;
    }
    const auto& position = snapshot.pose->pose().position();
    const double yaw = YawFromQuaternion(snapshot.pose->pose().orientation());
    const auto& twist = snapshot.velocity->twist();
    constexpr std::size_t kDefaultSize = 6;
    const std::size_t need =
        std::max(kDefaultSize, ExpectedElements(input, kDefaultSize));
    output->assign(need, 0.0f);

    double feat_x = position.x();
    double feat_y = position.y();
    double feat_yaw = yaw;
    if (relative_to_path_goal && snapshot.path != nullptr &&
        !snapshot.path->poses().empty()) {
        const auto& goal =
            snapshot.path->poses(snapshot.path->poses_size() - 1).pose();
        const double goal_yaw = YawFromQuaternion(goal.orientation());
        const double dx = goal.position().x() - position.x();
        const double dy = goal.position().y() - position.y();
        const double c = std::cos(-yaw);
        const double s = std::sin(-yaw);
        feat_x = c * dx - s * dy;
        feat_y = s * dx + c * dy;
        feat_yaw = std::remainder(goal_yaw - yaw, 2.0 * M_PI);
    }

    (*output)[0] = static_cast<float>(feat_x);
    (*output)[1] = static_cast<float>(feat_y);
    (*output)[2] = static_cast<float>(feat_yaw);
    (*output)[3] = static_cast<float>(twist.linear().x());
    (*output)[4] = static_cast<float>(twist.linear().y());
    (*output)[5] = static_cast<float>(twist.angular().z());
    return true;
}

}  // namespace policy_controller
}  // namespace controller
}  // namespace control
}  // namespace autonomy
