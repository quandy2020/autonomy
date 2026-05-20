// Copyright (c) 2018 Intel Corporation
// Copyright (c) 2019 Steven Macenski
// Copyright (c) 2019 Samsung Research America
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#pragma once

#include <memory>
#include <string>

#include "autonomy/commsgs/builtin_interfaces.hpp"
#include "autonomy/commsgs/geometry_msgs.hpp"
#include "autonomy/control/utils/odometry_utils.hpp"
#include "autonomy/transform/buffer.hpp"
#include "autonomy/transform/tf2/transform_datatypes.h"

namespace autonomy {
namespace tasks {
namespace utils {

/**
 * @brief get the current pose of the robot
 * @param global_pose Pose to transform
 * @param tf_buffer TF buffer to use for the transformation
 * @param global_frame Frame to transform into
 * @param robot_frame Frame to transform from
 * @param transform_timeout TF Timeout to use for transformation
 * @return bool Whether it could be transformed successfully
 */
bool getCurrentPose(commsgs::geometry_msgs::PoseStamped& global_pose,
                    std::shared_ptr<autonomy::transform::Buffer> tf_buffer,
                    const std::string global_frame = "map",
                    const std::string robot_frame = "base_link",
                    const float transform_timeout = 0.1f);

/**
 * @brief Resolve the robot pose in global_frame, preferring odometry over a
 * multi-hop TF lookup (map<-odom<-base_link).
 *
 * When odometry is published in the odom frame and map->odom is identity (mock
 * demo and many stacks), the odom pose is used directly. Otherwise a single-hop
 * global<-odom transform is applied before falling back to getCurrentPose().
 */
bool getGlobalRobotPose(
    commsgs::geometry_msgs::PoseStamped& global_pose,
    std::shared_ptr<autonomy::transform::Buffer> tf_buffer,
    std::shared_ptr<control::utils::OdomSmoother> odom_smoother,
    const std::string& global_frame, const std::string& robot_frame,
    float transform_timeout = 0.1f);

/**
 * @brief get an arbitrary pose in a target frame
 * @param input_pose Pose to transform
 * @param transformed_pose Output transformation
 * @param tf_buffer TF buffer to use for the transformation
 * @param target_frame Frame to transform into
 * @param transform_timeout TF Timeout to use for transformation
 * @return bool Whether it could be transformed successfully
 */
bool transformPoseInTargetFrame(
    const commsgs::geometry_msgs::PoseStamped& input_pose,
    commsgs::geometry_msgs::PoseStamped& transformed_pose,
    std::shared_ptr<autonomy::transform::Buffer> tf_buffer,
    const std::string target_frame, const float transform_timeout = 0.1f);

/**
 * @brief Obtains a transform from source_frame_id -> to target_frame_id
 * @param source_frame_id Source frame ID to convert from
 * @param target_frame_id Target frame ID to convert to
 * @param transform_tolerance Transform tolerance (in seconds)
 * @param tf_buffer TF buffer to use for the transformation
 * @param tf_transform Output source->target transform
 * @return True if got correct transform, otherwise false
 */
bool getTransform(const std::string& source_frame_id,
                  const std::string& target_frame_id,
                  const float transform_tolerance,
                  std::shared_ptr<autonomy::transform::Buffer> tf_buffer,
                  autonomy::transform::tf2::Transform& tf2_transform);

/**
 * @brief Obtains a transform from source_frame_id at source_time ->
 * to target_frame_id at target_time time
 * @param source_frame_id Source frame ID to convert from
 * @param source_time Source timestamp to convert from
 * @param target_frame_id Target frame ID to convert to
 * @param target_time Current node time to interpolate to
 * @param fixed_frame_id The frame in which to assume the transform is constant
 * in time
 * @param transform_tolerance Transform tolerance (in seconds)
 * @param tf_buffer TF buffer to use for the transformation
 * @param tf_transform Output source->target transform
 * @return True if got correct transform, otherwise false
 */
bool getTransform(const std::string& source_frame_id,
                  const commsgs::builtin_interfaces::Time& source_time,
                  const std::string& target_frame_id,
                  const commsgs::builtin_interfaces::Time& target_time,
                  const std::string& fixed_frame_id,
                  const float transform_tolerance,
                  std::shared_ptr<autonomy::transform::Buffer> tf_buffer,
                  autonomy::transform::tf2::Transform& tf2_transform);

/**
 * @brief Obtains a transform from source_frame_id -> to target_frame_id
 * @param source_frame_id Source frame ID to convert from
 * @param target_frame_id Target frame ID to convert to
 * @param transform_tolerance Transform tolerance (in seconds)
 * @param tf_buffer TF buffer to use for the transformation
 * @param transform_msg Output source->target transform msg
 * @return True if got correct transform, otherwise false
 */
bool getTransform(const std::string& source_frame_id,
                  const std::string& target_frame_id,
                  const float transform_tolerance,
                  std::shared_ptr<autonomy::transform::Buffer> tf_buffer,
                  commsgs::geometry_msgs::TransformStamped& transform_msg);

/**
 * @brief Obtains a transform from source_frame_id at source_time ->
 * to target_frame_id at target_time time
 * @param source_frame_id Source frame ID to convert from
 * @param source_time Source timestamp to convert from
 * @param target_frame_id Target frame ID to convert to
 * @param target_time Current node time to interpolate to
 * @param fixed_frame_id The frame in which to assume the transform is constant
 * in time
 * @param transform_tolerance Transform tolerance (in seconds)
 * @param tf_buffer TF buffer to use for the transformation
 * @param transform_msg Output source->target transform msg
 * @return True if got correct transform, otherwise false
 */
bool getTransform(const std::string& source_frame_id,
                  const commsgs::builtin_interfaces::Time& source_time,
                  const std::string& target_frame_id,
                  const commsgs::builtin_interfaces::Time& target_time,
                  const std::string& fixed_frame_id,
                  const float transform_tolerance,
                  std::shared_ptr<autonomy::transform::Buffer> tf_buffer,
                  commsgs::geometry_msgs::TransformStamped& transform_msg);

/**
 * @brief Validates a twist message contains no nans or infs
 * @param msg Twist message to validate
 * @return True if valid, false if contains unactionable values
 */
[[nodiscard]] bool validateTwist(const commsgs::geometry_msgs::Twist& msg);

}  // namespace utils
}  // namespace tasks
}  // namespace autonomy
