/*
 * Copyright 2025 The Openbot Authors (duyongquan)
 */

#include "autonomy/control/controller/policy_controller/observation/path_encoder.hpp"

#include <algorithm>
#include <cmath>

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

}  // namespace

bool EncodePath(const ObservationSnapshot& snapshot,
                const proto::ObservationInput& input,
                int path_point_count, std::vector<float>* output,
                std::string* error) {
    if (output == nullptr) {
        return false;
    }
    if (snapshot.path == nullptr || snapshot.path->poses().empty()) {
        if (error != nullptr) {
            *error = "Path modality requires a non-empty path";
        }
        return false;
    }
    const int count = std::max(1, path_point_count);
    output->assign(static_cast<std::size_t>(count * 2), 0.0f);

    double origin_x = 0.0;
    double origin_y = 0.0;
    double yaw = 0.0;
    if (snapshot.pose.has_value()) {
        origin_x = snapshot.pose->pose().position().x();
        origin_y = snapshot.pose->pose().position().y();
        yaw = YawFromQuaternion(snapshot.pose->pose().orientation());
    }
    const double cosine = std::cos(-yaw);
    const double sine = std::sin(-yaw);
    const int total = snapshot.path->poses_size();
    for (int i = 0; i < count; ++i) {
        const int source =
            total == 1 ? 0
                       : static_cast<int>(std::llround(
                             static_cast<double>(i) * (total - 1) /
                             static_cast<double>(count - 1)));
        const auto& waypoint = snapshot.path->poses(source).pose().position();
        const double delta_x = waypoint.x() - origin_x;
        const double delta_y = waypoint.y() - origin_y;
        (*output)[static_cast<std::size_t>(i * 2)] =
            static_cast<float>(cosine * delta_x - sine * delta_y);
        (*output)[static_cast<std::size_t>(i * 2 + 1)] =
            static_cast<float>(sine * delta_x + cosine * delta_y);
    }

    if (input.shape_size() > 0) {
        std::size_t need = 1;
        bool any = false;
        for (int i = 0; i < input.shape_size(); ++i) {
            if (input.shape(i) > 0) {
                need *= static_cast<std::size_t>(input.shape(i));
                any = true;
            }
        }
        if (any) {
            output->resize(need, 0.0f);
        }
    }
    return true;
}

}  // namespace policy_controller
}  // namespace controller
}  // namespace control
}  // namespace autonomy
