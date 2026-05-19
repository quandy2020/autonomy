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

#include "autonomy/common/log.hpp"
#include "autonomy/commsgs/builtin_interfaces.hpp"

namespace autonomy {
namespace transform {

TransformBroadcaster::TransformBroadcaster() = default;

void TransformBroadcaster::SendTransform(
    const commsgs::geometry_msgs::TransformStamped& transform) {
    std::vector<commsgs::geometry_msgs::TransformStamped> transforms;
    transforms.emplace_back(transform);
    SendTransform(transforms);
}

void TransformBroadcaster::SendTransform(
    const std::vector<commsgs::geometry_msgs::TransformStamped>& transforms) {
    transform_stampeds_.transforms = transforms;
    transform_stampeds_.header.stamp =
        commsgs::builtin_interfaces::Time::Now();
}

}  // namespace transform
}  // namespace autonomy
