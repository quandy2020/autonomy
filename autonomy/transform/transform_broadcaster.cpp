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

#include "autonomy/transform/transform_broadcaster.hpp"

#include "autonomy/common/logging.hpp"
#include <automsgs/msgs/builtin_interfaces/time.pb.h>
#include <automsgs/msgs/builtin_interfaces/duration.pb.h>
#include <automsgs/msgs/time_utils.hpp>

namespace autonomy {
namespace transform {

TransformBroadcaster::TransformBroadcaster() = default;

void TransformBroadcaster::SendTransform(
    const automsgs::msgs::geometry_msgs::TransformStamped& transform) {
    std::vector<automsgs::msgs::geometry_msgs::TransformStamped> transforms;
    transforms.emplace_back(transform);
    SendTransform(transforms);
}

void TransformBroadcaster::SendTransform(
    const std::vector<automsgs::msgs::geometry_msgs::TransformStamped>& transforms) {
    transform_stampeds_.clear_transforms();
    for (const auto& tf : transforms) {
        *transform_stampeds_.add_transforms() = tf;
    }
    *transform_stampeds_.mutable_header()->mutable_stamp() =
        automsgs::msgs::builtin_interfaces::TimeNow();
}

}  // namespace transform
}  // namespace autonomy
