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
 * @file pose_filter.cpp
 * @brief First-order pose filter (implementation).
 */

#include "autonomy/task/charging/dock/pose_filter.hpp"

#include <automsgs/msgs/time_utils.hpp>
#include "autonomy/task/charging/dock/geometry.hpp"

namespace autonomy {
namespace task {
namespace charging {

PoseFilter::PoseFilter(double coef, double timeout_sec)
    : coef_(coef), timeout_sec_(timeout_sec) {}

::automsgs::msgs::geometry_msgs::PoseStamped PoseFilter::Update(
    const ::automsgs::msgs::geometry_msgs::PoseStamped& measurement) {
    if (coef_ <= 0.0) {
        return measurement;
    }
    double dt = timeout_sec_ + 1.0;
    if (valid_) {
        dt = ::automsgs::msgs::builtin_interfaces::DurationToSeconds(
            measurement.header().stamp() - pose_.header().stamp());
        if (dt < 0.0) {
            dt = -dt;
        }
    }
    if (!valid_ || dt > timeout_sec_ ||
        pose_.header().frame_id() != measurement.header().frame_id()) {
        pose_ = measurement;
        valid_ = true;
        return pose_;
    }
    *pose_.mutable_header() = measurement.header();
    auto* p = pose_.mutable_pose()->mutable_position();
    const auto& m = measurement.pose().position();
    p->set_x((1.0 - coef_) * p->x() + coef_ * m.x());
    p->set_y((1.0 - coef_) * p->y() + coef_ * m.y());
    p->set_z((1.0 - coef_) * p->z() + coef_ * m.z());
    geom::Slerp(pose_.mutable_pose()->mutable_orientation(),
                measurement.pose().orientation(), coef_);
    return pose_;
}

}  // namespace charging
}  // namespace task
}  // namespace autonomy
