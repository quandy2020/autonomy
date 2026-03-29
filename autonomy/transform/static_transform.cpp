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
#include "autonomy/common/param_handler.hpp"
#include "autonomy/commsgs/builtin_interfaces.hpp"

namespace autonomy {
namespace transform {

StaticTransform::StaticTransform(
    const autonomy::transform::proto::TransformOptions& options,
    ::autolink::Node* node)
    : static_transform_options_(options), node_(node) {
    ::autolink::proto::RoleAttributes attr;
    attr.set_channel_name("/tf_static");
    attr.mutable_qos_profile()->CopyFrom(
        ::autolink::transport::QosProfileConf::QOS_PROFILE_TF_STATIC);
    writer_ =
        node_->CreateWriter<commsgs::geometry_msgs::TransformStampeds>(attr);
    SendTransforms();
}

void StaticTransform::SendTransforms() {
    const std::string& file_path = static_transform_options_.extrinsic_file();
    if (file_path.empty()) {
        AWARN << "StaticTransform: extrinsic_file is empty";
        return;
    }

    std::vector<commsgs::geometry_msgs::TransformStamped> transform_stamped_vec;
    if (ParseFromYaml(file_path, transform_stamped_vec)) {
        SendTransform(transform_stamped_vec);
    }
}

bool StaticTransform::ParseFromYaml(
    const std::string& file_path,
    std::vector<commsgs::geometry_msgs::TransformStamped>& transforms) {
    common::ParamHandler param_handler(file_path);
    if (!param_handler.FileOpenedSuccessfully()) {
        AERROR << "Extrinsic yaml file does not exist: " << file_path;
        return false;
    }

    try {
        YAML::Node config = param_handler.GetConfig();

        if (!config["static_transforms"]) {
            AWARN << "No 'static_transforms' section in: " << file_path;
            return false;
        }

        const YAML::Node& transforms_node = config["static_transforms"];
        if (!transforms_node.IsSequence()) {
            AERROR << "'static_transforms' should be a sequence in: "
                   << file_path;
            return false;
        }

        for (const auto& tf_node : transforms_node) {
            // Check enabled
            bool enabled = true;
            if (tf_node["enabled"]) {
                enabled = tf_node["enabled"].as<bool>();
            }
            if (!enabled) {
                continue;
            }

            commsgs::geometry_msgs::TransformStamped transform;

            // frame_id
            if (tf_node["frame_id"]) {
                transform.header.frame_id =
                    tf_node["frame_id"].as<std::string>();
            }

            // child_frame_id
            if (tf_node["child_frame_id"]) {
                transform.child_frame_id =
                    tf_node["child_frame_id"].as<std::string>();
            }

            // translation
            if (tf_node["translation"]) {
                const auto& trans = tf_node["translation"];
                if (trans["x"])
                    transform.transform.translation.x = trans["x"].as<double>();
                if (trans["y"])
                    transform.transform.translation.y = trans["y"].as<double>();
                if (trans["z"])
                    transform.transform.translation.z = trans["z"].as<double>();
            }

            // rotation
            if (tf_node["rotation"]) {
                const auto& rot = tf_node["rotation"];
                if (rot["x"])
                    transform.transform.rotation.x = rot["x"].as<double>();
                if (rot["y"])
                    transform.transform.rotation.y = rot["y"].as<double>();
                if (rot["z"])
                    transform.transform.rotation.z = rot["z"].as<double>();
                if (rot["w"])
                    transform.transform.rotation.w = rot["w"].as<double>();
            } else {
                transform.transform.rotation.w = 1.0;
            }

            transforms.push_back(transform);

            std::string name =
                tf_node["name"] ? tf_node["name"].as<std::string>() : "unnamed";
            AINFO << "Broadcast static transform '" << name << "': ["
                  << transform.header.frame_id << " -> "
                  << transform.child_frame_id << "]";
        }

    } catch (const std::exception& e) {
        AERROR << "Extrinsic yaml file parse failed: " << file_path
               << ", error: " << e.what();
        return false;
    }

    AINFO << "Loaded " << transforms.size()
          << " static transforms from: " << file_path;
    return !transforms.empty();
}

void StaticTransform::SendTransform(
    const std::vector<commsgs::geometry_msgs::TransformStamped>& msgtf) {
    for (const auto& new_tf : msgtf) {
        bool match_found = false;
        for (auto& existing_tf : transform_stampeds_.transforms) {
            if (new_tf.child_frame_id == existing_tf.child_frame_id) {
                existing_tf = new_tf;
                match_found = true;
                break;
            }
        }
        if (!match_found) {
            transform_stampeds_.transforms.push_back(new_tf);
        }
    }

    // Set timestamp
    transform_stampeds_.header.stamp = commsgs::builtin_interfaces::Time::Now();
    writer_->Write(transform_stampeds_);
}

}  // namespace transform
}  // namespace autonomy
