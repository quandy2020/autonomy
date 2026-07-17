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

#include "autonomy/transform/buffer_utils.hpp"

#include <glog/logging.h>

#include "autonomy/common/param_handler.hpp"
#include "autonomy/commsgs/builtin_interfaces.hpp"
#include "autonomy/transform/geometry_msgs/transform_stamped.h"

namespace autonomy {
namespace transform {

namespace {

geometry_msgs::TransformStamped ToTf2Message(
    const commsgs::geometry_msgs::TransformStamped& trans) {
    geometry_msgs::TransformStamped geo_msg;
    geo_msg.header.stamp =
        static_cast<uint64_t>(trans.header.stamp.sec) * 1000000000ULL +
        static_cast<uint64_t>(trans.header.stamp.nanosec);
    geo_msg.header.frame_id = trans.header.frame_id;
    geo_msg.child_frame_id = trans.child_frame_id;
    geo_msg.transform.translation.x = trans.transform.translation.x;
    geo_msg.transform.translation.y = trans.transform.translation.y;
    geo_msg.transform.translation.z = trans.transform.translation.z;
    geo_msg.transform.rotation.x = trans.transform.rotation.x;
    geo_msg.transform.rotation.y = trans.transform.rotation.y;
    geo_msg.transform.rotation.z = trans.transform.rotation.z;
    geo_msg.transform.rotation.w = trans.transform.rotation.w;
    return geo_msg;
}

}  // namespace

void ApplyTransformStampedToBuffer(
    Buffer* buffer, const commsgs::geometry_msgs::TransformStamped& trans,
    const std::string& authority, const bool is_static) {
    if (buffer == nullptr) {
        LOG(ERROR) << "ApplyTransformStampedToBuffer: buffer is null.";
        return;
    }
    try {
        buffer->setTransform(ToTf2Message(trans), authority, is_static);
    } catch (const std::exception& ex) {
        LOG(WARNING) << "Failed to apply transform [" << trans.header.frame_id
                     << " -> " << trans.child_frame_id << "]: " << ex.what();
    }
}

void ApplyStaticTransformsToBuffer(
    Buffer* buffer,
    const commsgs::geometry_msgs::TransformStampeds& transforms,
    const std::string& authority) {
    if (buffer == nullptr) {
        LOG(ERROR) << "ApplyStaticTransformsToBuffer: buffer is null.";
        return;
    }

    for (const auto& trans : transforms.transforms) {
        ApplyTransformStampedToBuffer(buffer, trans, authority, true);
    }
}

void ApplyTfMessageToBuffer(
    Buffer* buffer,
    const ::autonomy::commsgs::proto::tf2_msgs::TFMessage& message,
    const std::string& authority, const bool is_static) {
    if (buffer == nullptr) {
        return;
    }
    for (const auto& proto_tf : message.transforms()) {
        ApplyTransformStampedToBuffer(
            buffer, commsgs::geometry_msgs::FromProto(proto_tf), authority,
            is_static);
    }
}

bool ParseStaticTransformsFromYaml(
    const std::string& yaml_path,
    std::vector<commsgs::geometry_msgs::TransformStamped>& transforms) {
    transforms.clear();

    common::ParamHandler param_handler(yaml_path);
    if (!param_handler.FileOpenedSuccessfully()) {
        LOG(ERROR) << "Extrinsic yaml file does not exist: " << yaml_path;
        return false;
    }

    try {
        const YAML::Node& config = param_handler.GetConfig();
        if (!config["static_transforms"]) {
            LOG(WARNING) << "No static_transforms section in: " << yaml_path;
            return false;
        }

        const YAML::Node& transforms_node = config["static_transforms"];
        if (!transforms_node.IsSequence()) {
            LOG(ERROR) << "static_transforms must be a sequence in: "
                       << yaml_path;
            return false;
        }

        for (const auto& tf_node : transforms_node) {
            if (tf_node["enabled"] && !tf_node["enabled"].as<bool>()) {
                continue;
            }

            commsgs::geometry_msgs::TransformStamped transform;
            if (tf_node["frame_id"]) {
                transform.header.frame_id = tf_node["frame_id"].as<std::string>();
            }
            if (tf_node["child_frame_id"]) {
                transform.child_frame_id =
                    tf_node["child_frame_id"].as<std::string>();
            }
            if (tf_node["translation"]) {
                const auto& trans = tf_node["translation"];
                if (trans["x"]) {
                    transform.transform.translation.x = trans["x"].as<double>();
                }
                if (trans["y"]) {
                    transform.transform.translation.y = trans["y"].as<double>();
                }
                if (trans["z"]) {
                    transform.transform.translation.z = trans["z"].as<double>();
                }
            }
            if (tf_node["rotation"]) {
                const auto& rot = tf_node["rotation"];
                if (rot["x"]) {
                    transform.transform.rotation.x = rot["x"].as<double>();
                }
                if (rot["y"]) {
                    transform.transform.rotation.y = rot["y"].as<double>();
                }
                if (rot["z"]) {
                    transform.transform.rotation.z = rot["z"].as<double>();
                }
                if (rot["w"]) {
                    transform.transform.rotation.w = rot["w"].as<double>();
                }
            } else {
                transform.transform.rotation.w = 1.0;
            }

            transforms.push_back(transform);
        }
    } catch (const std::exception& e) {
        LOG(ERROR) << "Extrinsic yaml file parse failed: " << yaml_path
                   << ", error: " << e.what();
        return false;
    }

    return !transforms.empty();
}

proto::TransformOptions MakeTransformOptions(const std::string& yaml_path) {
    proto::TransformOptions options;
    options.set_extrinsic_file(yaml_path);
    return options;
}

int LoadStaticTransformsFromFile(Buffer* buffer, const std::string& yaml_path,
                                 const std::string& authority) {
    if (buffer == nullptr) {
        LOG(ERROR) << "LoadStaticTransformsFromFile: buffer is null.";
        return 0;
    }
    if (yaml_path.empty()) {
        LOG(WARNING) << "LoadStaticTransformsFromFile: yaml path is empty.";
        return 0;
    }

    commsgs::geometry_msgs::TransformStampeds transforms;
    if (!ParseStaticTransformsFromYaml(yaml_path, transforms.transforms)) {
        LOG(WARNING) << "No static transforms loaded from: " << yaml_path;
        return 0;
    }
    transforms.header.stamp = commsgs::builtin_interfaces::Time::Now();

    ApplyStaticTransformsToBuffer(buffer, transforms, authority);
    LOG(INFO) << "Loaded " << transforms.transforms.size()
              << " static transforms from: " << yaml_path;
    return static_cast<int>(transforms.transforms.size());
}

void SeedBenchmarkTfTree(Buffer* buffer, const std::string& authority) {
    if (buffer == nullptr) {
        return;
    }
    const auto stamp = commsgs::builtin_interfaces::Time::Now();
    auto make = [&stamp](const std::string& parent,
                         const std::string& child) {
        commsgs::geometry_msgs::TransformStamped t;
        t.header.stamp = stamp;
        t.header.frame_id = parent;
        t.child_frame_id = child;
        t.transform.rotation.w = 1.0;
        return t;
    };
    ApplyTransformStampedToBuffer(buffer, make("map", "odom"), authority, true);
    ApplyTransformStampedToBuffer(buffer, make("odom", "base_link"), authority,
                                  true);
}

}  // namespace transform
}  // namespace autonomy
