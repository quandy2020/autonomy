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

#include "autonomy/commsgs/builtin_interfaces.hpp"
#include "autonomy/transform/buffer_utils.hpp"
#include "autonomy/transform/transform_topics.hpp"

namespace autonomy {
namespace transform {

bool StaticTransformPublisher::LoadFromFile(const std::string& yaml_path) {
    transforms_.transforms.clear();
    if (!ParseStaticTransformsFromYaml(yaml_path, transforms_.transforms)) {
        return false;
    }
    transforms_.header.stamp = commsgs::builtin_interfaces::Time::Now();
    return true;
}

int StaticTransformPublisher::ApplyToBuffer(
    Buffer* buffer, const std::string& authority) {
    if (!IsLoaded()) {
        return 0;
    }
    ApplyStaticTransformsToBuffer(buffer, transforms_, authority);
    return static_cast<int>(transforms_.transforms.size());
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
            node->CreateWriter<commsgs::geometry_msgs::TransformStampeds>(
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

    LOG(INFO) << "Published " << transforms_.transforms.size()
              << " static transforms on " << kTfStaticTopic;
    return true;
}

}  // namespace transform
}  // namespace autonomy
