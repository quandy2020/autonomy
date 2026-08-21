/*
 * Copyright 2026 The Openbot Authors
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

#include "autonomy/task/common/transform_listener.hpp"

#include "autonomy/common/logging.hpp"
#include <automsgs/msgs/geometry_msgs/point.pb.h>
#include <automsgs/msgs/geometry_msgs/quaternion.pb.h>
#include <automsgs/msgs/geometry_msgs/pose.pb.h>
#include <automsgs/msgs/geometry_msgs/pose_stamped.pb.h>
#include <automsgs/msgs/geometry_msgs/transform.pb.h>
#include <automsgs/msgs/geometry_msgs/transform_stamped.pb.h>
#include <automsgs/msgs/geometry_msgs/twist.pb.h>
#include <automsgs/msgs/geometry_msgs/twist_stamped.pb.h>
#include <automsgs/msgs/geometry_msgs/vector3.pb.h>
#include "autonomy/transform/buffer.hpp"
#include "autonomy/transform/buffer_utils.hpp"
#include "autonomy/transform/transform_topics.hpp"

namespace autonomy {
namespace task {

namespace tf2_pb = ::automsgs::msgs::tf2_msgs;

TransformListener::~TransformListener() { Stop(); }

bool TransformListener::Start(const std::shared_ptr<autolink::Node>& node,
                               const std::string& transform_message_topic,
                               const std::string& transform_stampeds_topic) {
    Stop();
    if (!node) {
        return false;
    }

    auto* buffer = transform::Buffer::Instance();
    if (buffer->Init() != 0) {
        AWARN << "TransformListener: transform::Buffer::Init non-zero";
    }
    transform::SeedIdentityMapToOdom(buffer, "task_tf_seed");

    node_ = node;

    if (!transform_message_topic.empty()) {
        transform_message_reader_ = node_->CreateReader<tf2_pb::TFMessage>(
            transform_message_topic,
            [buffer](const std::shared_ptr<tf2_pb::TFMessage>& msg) {
                if (!msg) {
                    return;
                }
                transform::ApplyTfMessageToBuffer(buffer, *msg, "autolink_tf",
                                                  false);
            });
        if (!transform_message_reader_) {
            AERROR << "TransformListener: failed to subscribe "
                   << transform_message_topic;
            Stop();
            return false;
        }
        AINFO << "TransformListener: TFMessage on " << transform_message_topic;
    }

    if (!transform_stampeds_topic.empty()) {
        stampeds_reader_ =
            node_->CreateReader<automsgs::msgs::geometry_msgs::TransformStampeds>(
                transform_stampeds_topic,
                [buffer](const std::shared_ptr<
                         automsgs::msgs::geometry_msgs::TransformStampeds>& msg) {
                    if (!msg) {
                        return;
                    }
                    for (const auto& trans : msg->transforms()) {
                        transform::ApplyTransformStampedToBuffer(
                            buffer, trans, "autolink_tf", false);
                    }
                });
        if (!stampeds_reader_) {
            AWARN << "TransformListener: TransformStampeds subscribe failed: "
                  << transform_stampeds_topic;
        } else {
            AINFO << "TransformListener: TransformStampeds on "
                  << transform_stampeds_topic;
        }
    }

    return static_cast<bool>(transform_message_reader_ || stampeds_reader_);
}

void TransformListener::Stop() {
    transform_message_reader_.reset();
    stampeds_reader_.reset();
    node_.reset();
}

}  // namespace task
}  // namespace autonomy
