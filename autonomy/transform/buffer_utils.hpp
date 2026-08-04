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

#include <string>
#include <vector>

#include <automsgs/msgs/geometry_msgs/transform_stamped.pb.h>
#include <automsgs/msgs/geometry_msgs/pose_stamped.pb.h>
#include <automsgs/msgs/geometry_msgs/twist_stamped.pb.h>
#include <automsgs/msgs/tf2_msgs/tf_message.pb.h>
#include "autonomy/transform/buffer.hpp"
#include "autonomy/transform/proto/transform_options.pb.h"

namespace autonomy {
namespace transform {

/** Apply static transforms into the process-local TF buffer. */
void ApplyStaticTransformsToBuffer(
    Buffer* buffer,
    const automsgs::msgs::geometry_msgs::TransformStampeds& transforms,
    const std::string& authority);

/** Apply a single dynamic or static transform. */
void ApplyTransformStampedToBuffer(
    Buffer* buffer, const automsgs::msgs::geometry_msgs::TransformStamped& transform,
    const std::string& authority, bool is_static);

/** Apply all transforms from a tf2_msgs.TFMessage (Autolink /tf). */
void ApplyTfMessageToBuffer(
    Buffer* buffer,
    const automsgs::msgs::tf2_msgs::TFMessage& message,
    const std::string& authority, bool is_static = false);

/** Parse static_transforms section from extrinsic YAML. */
bool ParseStaticTransformsFromYaml(
    const std::string& yaml_path,
    std::vector<automsgs::msgs::geometry_msgs::TransformStamped>& transforms);

/**
 * Load static transforms from YAML and apply to buffer.
 * @return Number of transforms applied, or 0 on failure.
 */
int LoadStaticTransformsFromFile(Buffer* buffer, const std::string& yaml_path,
                                 const std::string& authority);

/** Build TransformOptions pointing at the given extrinsic YAML file. */
proto::TransformOptions MakeTransformOptions(const std::string& yaml_path);

/** Identity map→odom→base_link (matches fakedata static tree). */
void SeedBenchmarkTfTree(Buffer* buffer, const std::string& authority = "task_tf_seed");

}  // namespace transform
}  // namespace autonomy
