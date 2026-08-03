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

#include "autonomy/transform/static_transform_publisher.hpp"

#include <glog/logging.h>

#include <automsgs/msgs/builtin_interfaces/time.pb.h>
#include <automsgs/msgs/builtin_interfaces/duration.pb.h>
#include <automsgs/msgs/time_utils.hpp>
#include "autonomy/transform/buffer_utils.hpp"
#include "autonomy/transform/transform_topics.hpp"

namespace autonomy {
namespace transform {

bool StaticTransformPublisher::LoadFromFile(const std::string& yaml_path) {
    transforms_.clear_transforms();
    std::vector<automsgs::msgs::geometry_msgs::TransformStamped> parsed;
    if (!ParseStaticTransformsFromYaml(yaml_path, parsed)) {
        return false;
    }
    for (const auto& tf : parsed) {
        *transforms_.add_transforms() = tf;
    }
    *transforms_.mutable_header()->mutable_stamp() =
        automsgs::msgs::builtin_interfaces::TimeNow();
    return true;
}

int StaticTransformPublisher::ApplyToBuffer(
    Buffer* buffer, const std::string& authority) {
    if (!IsLoaded()) {
        return 0;
    }
    ApplyStaticTransformsToBuffer(buffer, transforms_, authority);
    return transforms_.transforms_size();
}

bool StaticTransformPublisher::Publish(std::shared_ptr<autolink::Node> node) {
    if (!node) {
        LOG(ERROR) << "StaticTransformPublisher::Publish requires a node.";
        return false;
    }
    if (!IsLoaded()) {
        LOG(WARNING) << "StaticTransformPublisher: nothing to publish.";
        return false;
    }

    if (!writer_) {
        writer_ =
            node->CreateWriter<automsgs::msgs::geometry_msgs::TransformStampeds>(
                kTfStaticTopic);
        if (!writer_) {
            LOG(ERROR) << "Failed to create tf_static writer.";
            return false;
        }
    }

    if (!writer_->Write(transforms_)) {
        LOG(ERROR) << "Failed to publish static transforms on " << kTfStaticTopic;
        return false;
    }

    LOG(INFO) << "Published " << transforms_.transforms_size()
              << " static transforms on " << kTfStaticTopic;
    return true;
}

}  // namespace transform
}  // namespace autonomy
