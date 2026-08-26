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

#pragma once

#include <memory>
#include <string>

#include "autolink/autolink.hpp"
#include "autonomy/common/macros.hpp"
#include <automsgs/msgs/geometry_msgs/transform_stamped.pb.h>
#include <automsgs/msgs/tf2_msgs/tf_message.pb.h>
#include "autonomy/transform/buffer.hpp"

namespace autonomy {
namespace transform {

/**
 * Loads static transforms from YAML and publishes them on /tf_static as
 * tf2_msgs.TFMessage (same wire type as autosim / Cartographer).
 */
class StaticTransformPublisher
{
public:
    AUTONOMY_SMART_PTR_DEFINITIONS(StaticTransformPublisher)

    StaticTransformPublisher() = default;

    /** Load transforms from extrinsic YAML. Returns false if none loaded. */
    bool LoadFromFile(const std::string& yaml_path);

    /** Inject loaded transforms into the TF buffer as static. */
    int ApplyToBuffer(Buffer* buffer,
                      const std::string& authority = "static_transform");

    /**
     * Publish loaded transforms on /tf_static (TFMessage).
     * Requires a valid autolink node with initialized transport.
     */
    bool Publish(std::shared_ptr<autolink::Node> node);

    const automsgs::msgs::geometry_msgs::TransformStampeds& GetTransformStampeds() const {
        return transforms_;
    }

    bool IsLoaded() const { return !transforms_.transforms().empty(); }

private:
    std::shared_ptr<autolink::Writer<automsgs::msgs::tf2_msgs::TFMessage>>
        writer_;
    automsgs::msgs::geometry_msgs::TransformStampeds transforms_;
};

}  // namespace transform
}  // namespace autonomy
