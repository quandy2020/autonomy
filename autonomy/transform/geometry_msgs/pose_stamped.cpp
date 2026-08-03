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

#include <automsgs/msgs/geometry_msgs/transform_stamped.pb.h>
#include <automsgs/msgs/geometry_msgs/pose_stamped.pb.h>
#include <automsgs/msgs/geometry_msgs/twist_stamped.pb.h>
#include <automsgs/msgs/geometry_msgs/pose_stamped.pb.h>
#include "autonomy/transform/geometry_msgs/transform_stamped.h"
#include "autonomy/transform/tf2/convert.h"
#include "autonomy/transform/tf2/utils.h"

namespace autonomy {
namespace transform {
namespace tf2 {

// Template specialization for getTimestamp
// Note: This returns a reference, but we need to convert from
// automsgs::msgs::builtin_interfaces::Time to tf2::Time (uint64_t). We use a static
// thread_local variable to store the converted value.
template <>
const Time& getTimestamp<automsgs::msgs::geometry_msgs::PoseStamped>(
    const automsgs::msgs::geometry_msgs::PoseStamped& t) {
    // Convert automsgs::msgs::builtin_interfaces::Time to tf2::Time (uint64_t
    // nanoseconds) Use thread_local static to ensure the reference remains
    // valid
    static thread_local Time cached_time = 0;
    cached_time = static_cast<uint64_t>(t.header().stamp().sec()) * 1000000000ULL +
                  static_cast<uint64_t>(t.header().stamp().nanosec());
    return cached_time;
}

// Template specialization for getFrameId
template <>
const std::string& getFrameId<automsgs::msgs::geometry_msgs::PoseStamped>(
    const automsgs::msgs::geometry_msgs::PoseStamped& t) {
    return t.header().frame_id();
}

// Template specialization for doTransform
template <>
void doTransform<automsgs::msgs::geometry_msgs::PoseStamped>(
    const automsgs::msgs::geometry_msgs::PoseStamped& data_in,
    automsgs::msgs::geometry_msgs::PoseStamped& data_out,
    const geometry_msgs::TransformStamped& transform) {
    // Keep the query stamp (e.g. Time{} = latest). Using the composed
    // transform's header.stamp can be slightly ahead of the newest dynamic TF
    // sample; reusing it for follow-up lookups causes extrapolation-into-future
    // and bogus poses when the mock odom thread publishes just behind the main
    // thread.
    *data_out.mutable_header()->mutable_stamp() = data_in.header().stamp();
    data_out.mutable_header()->set_frame_id(transform.header.frame_id);

    // Transform position
    data_out.mutable_pose()->mutable_position()->set_x(transform.transform.translation.x +
        (transform.transform.rotation.w * transform.transform.rotation.w +
         transform.transform.rotation.x * transform.transform.rotation.x -
         transform.transform.rotation.y * transform.transform.rotation.y -
         transform.transform.rotation.z * transform.transform.rotation.z) *
            data_in.pose().position().x() +
        2.0 *
            (transform.transform.rotation.x * transform.transform.rotation.y -
             transform.transform.rotation.w * transform.transform.rotation.z) *
            data_in.pose().position().y() +
        2.0 *
            (transform.transform.rotation.x * transform.transform.rotation.z +
             transform.transform.rotation.w * transform.transform.rotation.y) *
            data_in.pose().position().z());

    data_out.mutable_pose()->mutable_position()->set_y(transform.transform.translation.y +
        2.0 *
            (transform.transform.rotation.x * transform.transform.rotation.y +
             transform.transform.rotation.w * transform.transform.rotation.z) *
            data_in.pose().position().x() +
        (transform.transform.rotation.w * transform.transform.rotation.w -
         transform.transform.rotation.x * transform.transform.rotation.x +
         transform.transform.rotation.y * transform.transform.rotation.y -
         transform.transform.rotation.z * transform.transform.rotation.z) *
            data_in.pose().position().y() +
        2.0 *
            (transform.transform.rotation.y * transform.transform.rotation.z -
             transform.transform.rotation.w * transform.transform.rotation.x) *
            data_in.pose().position().z());

    data_out.mutable_pose()->mutable_position()->set_z(transform.transform.translation.z +
        2.0 *
            (transform.transform.rotation.x * transform.transform.rotation.z -
             transform.transform.rotation.w * transform.transform.rotation.y) *
            data_in.pose().position().x() +
        2.0 *
            (transform.transform.rotation.y * transform.transform.rotation.z +
             transform.transform.rotation.w * transform.transform.rotation.x) *
            data_in.pose().position().y() +
        (transform.transform.rotation.w * transform.transform.rotation.w -
         transform.transform.rotation.x * transform.transform.rotation.x -
         transform.transform.rotation.y * transform.transform.rotation.y +
         transform.transform.rotation.z * transform.transform.rotation.z) *
            data_in.pose().position().z());

    // Transform orientation (quaternion multiplication)
    // q_out = q_transform * q_in
    const auto& q_t = transform.transform.rotation;
    const auto& q_in = data_in.pose().orientation();

    data_out.mutable_pose()->mutable_orientation()->set_w(
        q_t.w * q_in.w() - q_t.x * q_in.x() - q_t.y * q_in.y() - q_t.z * q_in.z());
    data_out.mutable_pose()->mutable_orientation()->set_x(
        q_t.w * q_in.x() + q_t.x * q_in.w() + q_t.y * q_in.z() - q_t.z * q_in.y());
    data_out.mutable_pose()->mutable_orientation()->set_y(
        q_t.w * q_in.y() - q_t.x * q_in.z() + q_t.y * q_in.w() + q_t.z * q_in.x());
    data_out.mutable_pose()->mutable_orientation()->set_z(
        q_t.w * q_in.z() + q_t.x * q_in.y() - q_t.y * q_in.x() + q_t.z * q_in.w());
}

// fromMsg specialization is now in pose_stamped.h header file

}  // namespace tf2
}  // namespace transform
}  // namespace autonomy
