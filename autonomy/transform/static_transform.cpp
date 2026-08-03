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

#include "autonomy/transform/static_transform.hpp"

#include "autonomy/common/logging.hpp"
#include <automsgs/msgs/builtin_interfaces/time.pb.h>
#include <automsgs/msgs/builtin_interfaces/duration.pb.h>
#include <automsgs/msgs/time_utils.hpp>
#include "autonomy/transform/buffer_utils.hpp"

namespace autonomy {
namespace transform {

StaticTransform::StaticTransform(
    const autonomy::transform::proto::TransformOptions& options)
    : static_transform_options_(options) {
    SendTransforms();
}

void StaticTransform::SendTransforms() {
    const std::string& file_path = static_transform_options_.extrinsic_file();
    if (file_path.empty()) {
        AWARN << "StaticTransform: extrinsic_file is empty";
        return;
    }

    std::vector<automsgs::msgs::geometry_msgs::TransformStamped> transform_stamped_vec;
    if (ParseFromYaml(file_path, transform_stamped_vec)) {
        SendTransform(transform_stamped_vec);
    }
}

bool StaticTransform::ParseFromYaml(
    const std::string& file_path,
    std::vector<automsgs::msgs::geometry_msgs::TransformStamped>& transforms) {
    if (!ParseStaticTransformsFromYaml(file_path, transforms)) {
        return false;
    }

    for (const auto& transform : transforms) {
        AINFO << "Broadcast static transform: [" << transform.header().frame_id()
              << " -> " << transform.child_frame_id() << "]";
    }

    AINFO << "Loaded " << transforms.size()
          << " static transforms from: " << file_path;
    return true;
}

void StaticTransform::SendTransform(
    const std::vector<automsgs::msgs::geometry_msgs::TransformStamped>& msgtf) {
    for (const auto& new_tf : msgtf) {
        bool match_found = false;
        for (auto& existing_tf : *transform_stampeds_.mutable_transforms()) {
            if (new_tf.child_frame_id() == existing_tf.child_frame_id()) {
                existing_tf = new_tf;
                match_found = true;
                break;
            }
        }
        if (!match_found) {
            *transform_stampeds_.add_transforms() = new_tf;
        }
    }

    *transform_stampeds_.mutable_header()->mutable_stamp() =
        automsgs::msgs::builtin_interfaces::TimeNow();
}

}  // namespace transform
}  // namespace autonomy
