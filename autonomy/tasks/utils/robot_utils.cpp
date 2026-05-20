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

#include "autonomy/tasks/utils/robot_utils.hpp"

#include <cmath>
#include <memory>
#include <string>

#include "autonomy/common/logging.hpp"
#include "autonomy/commsgs/builtin_interfaces.hpp"
#include "autonomy/transform/geometry_msgs/transform_stamped.h"
#include "autonomy/transform/tf2/buffer_core.h"
#include "autonomy/transform/tf2/convert.h"
#include "autonomy/transform/tf2/transform_datatypes.h"

using autonomy::transform::tf2::fromMsg;
using autonomy::transform::tf2::Transform;
using autonomy::transform::tf2::TransformException;

namespace autonomy {
namespace tasks {
namespace utils {

using autonomy::transform::tf2::BufferCore;

bool getCurrentPose(commsgs::geometry_msgs::PoseStamped& global_pose,
                    std::shared_ptr<autonomy::transform::Buffer> tf_buffer,
                    const std::string global_frame,
                    const std::string robot_frame,
                    const float transform_timeout) {
    if (!tf_buffer) {
        AERROR << "TF buffer is null";
        return false;
    }

    // Wait using Buffer::canTransform, then read the raw tf2 geometry message.
    // Buffer::lookupTransform + TF2MsgToConvert has produced bogus translation.x
    // (float Vector3 in commsgs vs double in internal geometry_msgs).
    try {
        const commsgs::builtin_interfaces::Time latest{};
        std::string err;
        if (!tf_buffer->canTransform(global_frame, robot_frame, latest,
                                     transform_timeout, &err)) {
            AERROR << "canTransform failed for " << robot_frame << " in "
                   << global_frame << ": " << err;
            return false;
        }
        const geometry_msgs::TransformStamped gt =
            static_cast<BufferCore&>(*tf_buffer).lookupTransform(
                global_frame, robot_frame, 0ULL);
        global_pose.header.frame_id = global_frame;
        global_pose.header.stamp = latest;
        global_pose.pose.position.x = gt.transform.translation.x;
        global_pose.pose.position.y = gt.transform.translation.y;
        global_pose.pose.position.z = gt.transform.translation.z;
        global_pose.pose.orientation.x = gt.transform.rotation.x;
        global_pose.pose.orientation.y = gt.transform.rotation.y;
        global_pose.pose.orientation.z = gt.transform.rotation.z;
        global_pose.pose.orientation.w = gt.transform.rotation.w;
        return true;
    } catch (const TransformException& ex) {
        AERROR << "Transform error looking up robot pose: " << ex.what();
        AERROR << "Failed to lookup " << robot_frame << " in " << global_frame
               << ": " << ex.what();
    } catch (const std::exception& ex) {
        AERROR << "Failed to lookup " << robot_frame << " in " << global_frame
               << ": " << ex.what();
    }

    return false;
}

bool transformPoseInTargetFrame(
    const commsgs::geometry_msgs::PoseStamped& input_pose,
    commsgs::geometry_msgs::PoseStamped& transformed_pose,
    std::shared_ptr<autonomy::transform::Buffer> tf_buffer,
    const std::string target_frame, const float transform_timeout) {
    if (!tf_buffer) {
        AERROR << "TF buffer is null";
        return false;
    }

    try {
        transformed_pose =
            tf_buffer->transform(input_pose, target_frame, transform_timeout);
        return true;
    } catch (const TransformException& ex) {
        AERROR << "Transform error looking up target frame: " << ex.what();
        AERROR << "Failed to transform from " << input_pose.header.frame_id
               << " to " << target_frame << ": " << ex.what();
    } catch (const std::exception& ex) {
        AERROR << "Failed to transform from " << input_pose.header.frame_id
               << " to " << target_frame << ": " << ex.what();
    }

    return false;
}

bool getTransform(const std::string& source_frame_id,
                  const std::string& target_frame_id,
                  const float transform_tolerance,
                  std::shared_ptr<autonomy::transform::Buffer> tf_buffer,
                  commsgs::geometry_msgs::TransformStamped& transform_msg) {
    if (!tf_buffer) {
        AERROR << "TF buffer is null";
        return false;
    }

    if (source_frame_id == target_frame_id) {
        // We are already in required frame
        transform_msg.header.frame_id = target_frame_id;
        transform_msg.child_frame_id = source_frame_id;
        transform_msg.transform.translation.x = 0.0;
        transform_msg.transform.translation.y = 0.0;
        transform_msg.transform.translation.z = 0.0;
        transform_msg.transform.rotation.x = 0.0;
        transform_msg.transform.rotation.y = 0.0;
        transform_msg.transform.rotation.z = 0.0;
        transform_msg.transform.rotation.w = 1.0;
        transform_msg.header.stamp = commsgs::builtin_interfaces::Time::Now();
        return true;
    }

    try {
        // Obtaining the transform to get data from source to target frame
        commsgs::builtin_interfaces::Time zero_time;
        zero_time.sec = 0;
        zero_time.nanosec = 0;
        transform_msg = tf_buffer->lookupTransform(
            target_frame_id, source_frame_id, zero_time, transform_tolerance);
        return true;
    } catch (const TransformException& e) {
        AERROR << "Failed to get \"" << source_frame_id << "\"->\""
               << target_frame_id << "\" frame transform: " << e.what();
        return false;
    } catch (const std::exception& e) {
        AERROR << "Failed to get \"" << source_frame_id << "\"->\""
               << target_frame_id << "\" frame transform: " << e.what();
        return false;
    }
}

bool getTransform(const std::string& source_frame_id,
                  const std::string& target_frame_id,
                  const float transform_tolerance,
                  std::shared_ptr<autonomy::transform::Buffer> tf_buffer,
                  Transform& tf2_transform) {
    tf2_transform.setIdentity();  // initialize by identical transform
    commsgs::geometry_msgs::TransformStamped transform;
    if (getTransform(source_frame_id, target_frame_id, transform_tolerance,
                     tf_buffer, transform)) {
        // Convert TransformStamped to TF2 transform
        geometry_msgs::Transform tf2_transform_msg;
        tf2_transform_msg.translation.x = transform.transform.translation.x;
        tf2_transform_msg.translation.y = transform.transform.translation.y;
        tf2_transform_msg.translation.z = transform.transform.translation.z;
        tf2_transform_msg.rotation.x = transform.transform.rotation.x;
        tf2_transform_msg.rotation.y = transform.transform.rotation.y;
        tf2_transform_msg.rotation.z = transform.transform.rotation.z;
        tf2_transform_msg.rotation.w = transform.transform.rotation.w;
        // Use direct construction instead of transformMsgToTF2
        using namespace autonomy::transform::tf2;
        tf2_transform = Transform(Quaternion(tf2_transform_msg.rotation.x,
                                             tf2_transform_msg.rotation.y,
                                             tf2_transform_msg.rotation.z,
                                             tf2_transform_msg.rotation.w),
                                  Vector3(tf2_transform_msg.translation.x,
                                          tf2_transform_msg.translation.y,
                                          tf2_transform_msg.translation.z));
        return true;
    }
    return false;
}

bool getTransform(const std::string& source_frame_id,
                  const commsgs::builtin_interfaces::Time& source_time,
                  const std::string& target_frame_id,
                  const commsgs::builtin_interfaces::Time& target_time,
                  const std::string& fixed_frame_id,
                  const float transform_tolerance,
                  std::shared_ptr<autonomy::transform::Buffer> tf_buffer,
                  commsgs::geometry_msgs::TransformStamped& transform_msg) {
    if (!tf_buffer) {
        AERROR << "TF buffer is null";
        return false;
    }

    try {
        // Obtaining the transform to get data from source to target frame.
        // This also considers the time shift between source and target.
        transform_msg = tf_buffer->lookupTransform(
            target_frame_id, target_time, source_frame_id, source_time,
            fixed_frame_id, transform_tolerance);
        return true;
    } catch (const TransformException& ex) {
        AERROR << "Failed to get \"" << source_frame_id << "\"->\""
               << target_frame_id << "\" frame transform: " << ex.what();
        return false;
    } catch (const std::exception& ex) {
        AERROR << "Failed to get \"" << source_frame_id << "\"->\""
               << target_frame_id << "\" frame transform: " << ex.what();
        return false;
    }
    return false;  // Should never reach here, but satisfy compiler
}

bool getTransform(const std::string& source_frame_id,
                  const commsgs::builtin_interfaces::Time& source_time,
                  const std::string& target_frame_id,
                  const commsgs::builtin_interfaces::Time& target_time,
                  const std::string& fixed_frame_id,
                  const float transform_tolerance,
                  std::shared_ptr<autonomy::transform::Buffer> tf_buffer,
                  Transform& tf2_transform) {
    commsgs::geometry_msgs::TransformStamped transform;
    tf2_transform.setIdentity();  // initialize by identical transform
    if (getTransform(source_frame_id, source_time, target_frame_id, target_time,
                     fixed_frame_id, transform_tolerance, tf_buffer,
                     transform)) {
        // Convert TransformStamped to TF2 transform
        geometry_msgs::Transform tf2_transform_msg;
        tf2_transform_msg.translation.x = transform.transform.translation.x;
        tf2_transform_msg.translation.y = transform.transform.translation.y;
        tf2_transform_msg.translation.z = transform.transform.translation.z;
        tf2_transform_msg.rotation.x = transform.transform.rotation.x;
        tf2_transform_msg.rotation.y = transform.transform.rotation.y;
        tf2_transform_msg.rotation.z = transform.transform.rotation.z;
        tf2_transform_msg.rotation.w = transform.transform.rotation.w;
        // Use direct construction instead of transformMsgToTF2
        using namespace autonomy::transform::tf2;
        tf2_transform = Transform(Quaternion(tf2_transform_msg.rotation.x,
                                             tf2_transform_msg.rotation.y,
                                             tf2_transform_msg.rotation.z,
                                             tf2_transform_msg.rotation.w),
                                  Vector3(tf2_transform_msg.translation.x,
                                          tf2_transform_msg.translation.y,
                                          tf2_transform_msg.translation.z));
        return true;
    }

    return false;
}

bool validateTwist(const commsgs::geometry_msgs::Twist& msg) {
    if (std::isinf(msg.linear.x) || std::isnan(msg.linear.x)) {
        return false;
    }

    if (std::isinf(msg.linear.y) || std::isnan(msg.linear.y)) {
        return false;
    }

    if (std::isinf(msg.linear.z) || std::isnan(msg.linear.z)) {
        return false;
    }

    if (std::isinf(msg.angular.x) || std::isnan(msg.angular.x)) {
        return false;
    }

    if (std::isinf(msg.angular.y) || std::isnan(msg.angular.y)) {
        return false;
    }

    if (std::isinf(msg.angular.z) || std::isnan(msg.angular.z)) {
        return false;
    }

    return true;
}

}  // namespace utils
}  // namespace tasks
}  // namespace autonomy
