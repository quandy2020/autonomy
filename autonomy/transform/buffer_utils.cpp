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
#include <automsgs/msgs/builtin_interfaces/time.pb.h>
#include <automsgs/msgs/builtin_interfaces/duration.pb.h>
#include <automsgs/msgs/time_utils.hpp>
#include <automsgs/msgs/tf2_msgs/tf_message.pb.h>
#include <automsgs/msgs/geometry_msgs/transform_stamped.pb.h>
#include "autonomy/transform/geometry_msgs/transform_stamped.h"

namespace autonomy {
namespace transform {

namespace {

geometry_msgs::TransformStamped ToTf2Message(
    const automsgs::msgs::geometry_msgs::TransformStamped& trans) {
    geometry_msgs::TransformStamped geo_msg;
    geo_msg.header.stamp = static_cast<uint64_t>(trans.header().stamp().sec()) * 1000000000ULL +
        static_cast<uint64_t>(trans.header().stamp().nanosec());
    geo_msg.header.frame_id = trans.header().frame_id();
    geo_msg.child_frame_id = trans.child_frame_id();
    geo_msg.transform.translation.x = trans.transform().translation().x();
    geo_msg.transform.translation.y = trans.transform().translation().y();
    geo_msg.transform.translation.z = trans.transform().translation().z();
    geo_msg.transform.rotation.x = trans.transform().rotation().x();
    geo_msg.transform.rotation.y = trans.transform().rotation().y();
    geo_msg.transform.rotation.z = trans.transform().rotation().z();
    geo_msg.transform.rotation.w = trans.transform().rotation().w();
    return geo_msg;
}

}  // namespace

void ApplyTransformStampedToBuffer(
    Buffer* buffer, const automsgs::msgs::geometry_msgs::TransformStamped& trans,
    const std::string& authority, const bool is_static) {
    if (buffer == nullptr) {
        LOG(ERROR) << "ApplyTransformStampedToBuffer: buffer is null.";
        return;
    }
    try {
        buffer->setTransform(ToTf2Message(trans), authority, is_static);
    } catch (const std::exception& ex) {
        LOG(WARNING) << "Failed to apply transform [" << trans.header().frame_id()
                     << " -> " << trans.child_frame_id() << "]: " << ex.what();
    }
}

void ApplyStaticTransformsToBuffer(
    Buffer* buffer,
    const automsgs::msgs::geometry_msgs::TransformStampeds& transforms,
    const std::string& authority) {
    if (buffer == nullptr) {
        LOG(ERROR) << "ApplyStaticTransformsToBuffer: buffer is null.";
        return;
    }

    for (const auto& trans : transforms.transforms()) {
        ApplyTransformStampedToBuffer(buffer, trans, authority, true);
    }
}

void ApplyTfMessageToBuffer(
    Buffer* buffer,
    const automsgs::msgs::tf2_msgs::TFMessage& message,
    const std::string& authority, const bool is_static) {
    if (buffer == nullptr) {
        return;
    }
    for (const auto& proto_tf : message.transforms()) {
        // Mixed /tf batches from autosim: odom→base and map→odom stay
        // dynamic; URDF mounts must be static or laser→map lookups fail
        // when scan stamps fall outside the short dynamic cache.
        const std::string& parent = proto_tf.header().frame_id();
        const bool edge_static =
            is_static || (parent != "odom" && parent != "map");
        ApplyTransformStampedToBuffer(buffer, proto_tf, authority, edge_static);
    }
}

bool ParseStaticTransformsFromYaml(
    const std::string& yaml_path,
    std::vector<automsgs::msgs::geometry_msgs::TransformStamped>& transforms) {
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

            automsgs::msgs::geometry_msgs::TransformStamped transform;
            if (tf_node["frame_id"]) {
                transform.mutable_header()->set_frame_id(tf_node["frame_id"].as<std::string>());
            }
            if (tf_node["child_frame_id"]) {
                transform.set_child_frame_id(tf_node["child_frame_id"].as<std::string>());
            }
            if (tf_node["translation"]) {
                const auto& trans = tf_node["translation"];
                if (trans["x"]) {
                    transform.mutable_transform()->mutable_translation()->set_x(trans["x"].as<double>());
                }
                if (trans["y"]) {
                    transform.mutable_transform()->mutable_translation()->set_y(trans["y"].as<double>());
                }
                if (trans["z"]) {
                    transform.mutable_transform()->mutable_translation()->set_z(trans["z"].as<double>());
                }
            }
            if (tf_node["rotation"]) {
                const auto& rot = tf_node["rotation"];
                if (rot["x"]) {
                    transform.mutable_transform()->mutable_rotation()->set_x(rot["x"].as<double>());
                }
                if (rot["y"]) {
                    transform.mutable_transform()->mutable_rotation()->set_y(rot["y"].as<double>());
                }
                if (rot["z"]) {
                    transform.mutable_transform()->mutable_rotation()->set_z(rot["z"].as<double>());
                }
                if (rot["w"]) {
                    transform.mutable_transform()->mutable_rotation()->set_w(rot["w"].as<double>());
                }
            } else {
                transform.mutable_transform()->mutable_rotation()->set_w(1.0);
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

    automsgs::msgs::geometry_msgs::TransformStampeds transforms;
    std::vector<automsgs::msgs::geometry_msgs::TransformStamped> parsed;
    if (!ParseStaticTransformsFromYaml(yaml_path, parsed)) {
        LOG(WARNING) << "No static transforms loaded from: " << yaml_path;
        return 0;
    }
    for (const auto& tf : parsed) {
        *transforms.mutable_transforms()->Add() = tf;
    }
    *transforms.mutable_header()->mutable_stamp() =
        automsgs::msgs::builtin_interfaces::TimeNow();

    ApplyStaticTransformsToBuffer(buffer, transforms, authority);
    LOG(INFO) << "Loaded " << transforms.transforms_size()
              << " static transforms from: " << yaml_path;
    return transforms.transforms_size();
}

void SeedBenchmarkTfTree(Buffer* buffer, const std::string& authority) {
    if (buffer == nullptr) {
        return;
    }
    const auto stamp = automsgs::msgs::builtin_interfaces::TimeNow();
    auto make = [&stamp](const std::string& parent,
                         const std::string& child) {
        automsgs::msgs::geometry_msgs::TransformStamped t;
        *t.mutable_header()->mutable_stamp() = stamp;
        t.mutable_header()->set_frame_id(parent);
        t.set_child_frame_id(child);
        t.mutable_transform()->mutable_rotation()->set_w(1.0);
        return t;
    };
    ApplyTransformStampedToBuffer(buffer, make("map", "odom"), authority, true);
    ApplyTransformStampedToBuffer(buffer, make("odom", "base_link"), authority,
                                  true);
}

void SeedIdentityMapToOdom(Buffer* buffer, const std::string& authority) {
    if (buffer == nullptr) {
        return;
    }
    const auto stamp = automsgs::msgs::builtin_interfaces::TimeNow();
    auto make = [&stamp](const std::string& parent,
                         const std::string& child) {
        automsgs::msgs::geometry_msgs::TransformStamped t;
        *t.mutable_header()->mutable_stamp() = stamp;
        t.mutable_header()->set_frame_id(parent);
        t.set_child_frame_id(child);
        t.mutable_transform()->mutable_rotation()->set_w(1.0);
        return t;
    };
    // Seed a single connected tree so canTransform(map, base_link) works before
    // the first live /tf tick. Live odom→base_footprint overwrites the seed.
    ApplyTransformStampedToBuffer(buffer, make("map", "odom"), authority, true);
    ApplyTransformStampedToBuffer(buffer, make("odom", "base_footprint"),
                                  authority, true);
    ApplyTransformStampedToBuffer(buffer, make("base_footprint", "base_link"),
                                  authority, true);
}

}  // namespace transform
}  // namespace autonomy
