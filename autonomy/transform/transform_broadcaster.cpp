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

#include "autolink/common/log.hpp"

namespace autonomy {
namespace transform {

TransformBroadcaster::TransformBroadcaster(const std::shared_ptr<::autolink::Node>& node) : node_(node) {
    if (!node_) {
        AERROR << "TransformBroadcaster: Node is null.";
        return;
    }
    ::autolink::proto::RoleAttributes attr;
    attr.set_channel_name("/tf");
    writer_ = node_->CreateWriter<commsgs::geometry_msgs::TransformStampeds>(attr);
}

TransformBroadcaster::TransformBroadcaster(::autolink::Node* node) {
    if (!node) {
        AERROR << "TransformBroadcaster: Node pointer is null.";
        return;
    }
    ::autolink::proto::RoleAttributes attr;
    attr.set_channel_name("/tf");
    writer_ = node->CreateWriter<commsgs::geometry_msgs::TransformStampeds>(attr);
}

void TransformBroadcaster::SendTransform(const commsgs::geometry_msgs::TransformStamped& transform) {
    std::vector<commsgs::geometry_msgs::TransformStamped> transforms;
    transforms.emplace_back(transform);
    SendTransform(transforms);
}

void TransformBroadcaster::SendTransform(const std::vector<commsgs::geometry_msgs::TransformStamped>& transforms) {
    if (!writer_) {
        AERROR << "TransformBroadcaster: Writer is null, cannot send transform.";
        return;
    }
    auto message = std::make_shared<commsgs::geometry_msgs::TransformStampeds>();
    message->transforms = transforms;
    writer_->Write(message);
}
}  // namespace transform
}  // namespace autonomy
