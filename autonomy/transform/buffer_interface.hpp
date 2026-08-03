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

#pragma once

#include <algorithm>
#include <string>

#include <automsgs/msgs/builtin_interfaces/time.pb.h>
#include <automsgs/msgs/builtin_interfaces/duration.pb.h>
#include <automsgs/msgs/time_utils.hpp>
#include <automsgs/msgs/geometry_msgs/transform_stamped.pb.h>
#include <automsgs/msgs/geometry_msgs/pose_stamped.pb.h>
#include <automsgs/msgs/geometry_msgs/twist_stamped.pb.h>
#include <automsgs/msgs/geometry_msgs/transform_stamped.pb.h>
#include "autonomy/transform/geometry_msgs/transform_stamped.h"
#include "autonomy/transform/tf2/convert.h"
#include "autonomy/transform/tf2/time.h"

namespace autonomy {
namespace transform {

// extend the TFCore class and the TFCpp class
class BufferInterface
{
public:
    /** \brief Get the transform between two frames by frame ID.
     * \param target_frame The frame to which data should be transformed
     * \param source_frame The frame where the data originated
     * \param time The time at which the value of the transform is desired. (0
     *will get the latest)
     * \param timeout How long to block before failing
     * \return The transform between the frames
     *
     * Possible exceptions tf2::LookupException, tf2::ConnectivityException,
     * tf2::ExtrapolationException, tf2::InvalidArgumentException
     */
    virtual automsgs::msgs::geometry_msgs::TransformStamped lookupTransform(
        const std::string& target_frame, const std::string& source_frame,
        const automsgs::msgs::builtin_interfaces::Time& time,
        const float timeout_second = 0.01f) const = 0;

    /** \brief Get the transform between two frames by frame ID assuming fixed
     *frame.
     * \param target_frame The frame to which data should be transformed
     * \param target_time The time to which the data should be transformed. (0
     *will get the latest)
     * \param source_frame The frame where the data originated
     * \param source_time The time at which the source_frame should be
     *evaluated. (0 will get the latest)
     * \param fixed_frame The frame in which to assume the transform is constant
     *in time.
     * \param timeout How long to block before failing
     * \return The transform between the frames
     *
     * Possible exceptions tf2::LookupException, tf2::ConnectivityException,
     * tf2::ExtrapolationException, tf2::InvalidArgumentException
     */
    virtual automsgs::msgs::geometry_msgs::TransformStamped lookupTransform(
        const std::string& target_frame,
        const automsgs::msgs::builtin_interfaces::Time& target_time,
        const std::string& source_frame,
        const automsgs::msgs::builtin_interfaces::Time& source_time,
        const std::string& fixed_frame,
        const float timeout_second = 0.01f) const = 0;

    /** \brief Test if a transform is possible
     * \param target_frame The frame into which to transform
     * \param source_frame The frame from which to transform
     * \param time The time at which to transform
     * \param timeout How long to block before failing
     * \param errstr A pointer to a string which will be filled with why the
     * transform failed, if not nullptr
     * \return True if the transform is possible, false otherwise
     */
    virtual bool canTransform(const std::string& target_frame,
                              const std::string& source_frame,
                              const automsgs::msgs::builtin_interfaces::Time& time,
                              const float timeout_second = 0.01f,
                              std::string* errstr = nullptr) const = 0;

    /** \brief Test if a transform is possible
     * \param target_frame The frame into which to transform
     * \param target_time The time into which to transform
     * \param source_frame The frame from which to transform
     * \param source_time The time from which to transform
     * \param fixed_frame The frame in which to treat the transform as constant
     * in time
     * \param timeout How long to block before failing
     * \param errstr A pointer to a string which will be filled with why the
     * transform failed, if not nullptr
     * \return True if the transform is possible, false otherwise
     */
    virtual bool canTransform(
        const std::string& target_frame,
        const automsgs::msgs::builtin_interfaces::Time& target_time,
        const std::string& source_frame,
        const automsgs::msgs::builtin_interfaces::Time& source_time,
        const std::string& fixed_frame, const float timeout_second = 0.01f,
        std::string* errstr = nullptr) const = 0;

    // Transform, simple api, with pre-allocation
    template <typename T>
    T& transform(const T& in, T& out,
                 const std::string& target_frame,  // NOLINT
                 float timeout = 0.0f) const {
        // Convert tf2::Time (uint64_t nanoseconds) to
        // automsgs::msgs::builtin_interfaces::Time
        // Copy timestamp by value: PoseStamped getTimestamp uses thread-local
        // storage; holding a reference across lookupTransform is unsafe.
        const tf2::Time tf2_time = tf2::getTimestamp(in);
        automsgs::msgs::builtin_interfaces::Time stamp;
        stamp.set_sec(static_cast<int32_t>(tf2_time / 1'000'000'000));
        stamp.set_nanosec(static_cast<uint32_t>(tf2_time % 1'000'000'000));
        // Get transform in commsgs format
        const auto& commsgs_transform =
            lookupTransform(target_frame, tf2::getFrameId(in), stamp, timeout);
        // Convert automsgs::msgs::geometry_msgs::TransformStamped to
        // geometry_msgs::TransformStamped
        geometry_msgs::TransformStamped tf2_transform;
        tf2_transform.header.stamp =
            static_cast<uint64_t>(commsgs_transform.header().stamp().sec()) *
                1000000000ULL +
            static_cast<uint64_t>(commsgs_transform.header().stamp().nanosec());
        tf2_transform.header.frame_id = commsgs_transform.header().frame_id();
        tf2_transform.child_frame_id = commsgs_transform.child_frame_id();
        tf2_transform.transform.translation.x =
            commsgs_transform.transform().translation().x();
        tf2_transform.transform.translation.y =
            commsgs_transform.transform().translation().y();
        tf2_transform.transform.translation.z =
            commsgs_transform.transform().translation().z();
        tf2_transform.transform.rotation.x =
            commsgs_transform.transform().rotation().x();
        tf2_transform.transform.rotation.y =
            commsgs_transform.transform().rotation().y();
        tf2_transform.transform.rotation.z =
            commsgs_transform.transform().rotation().z();
        tf2_transform.transform.rotation.w =
            commsgs_transform.transform().rotation().w();
        // do the transform
        tf2::doTransform(in, out, tf2_transform);
        return out;
    }

    // transform, simple api, no pre-allocation
    template <typename T>
    T transform(const T& in, const std::string& target_frame,
                float timeout = 0.0f) const {
        T out;
        return transform(in, out, target_frame, timeout);
    }

    // transform, simple api, different types, pre-allocation
    template <typename A, typename B>
    B& transform(const A& in, B& out,
                 const std::string& target_frame,  // NOLINT
                 float timeout = 0.0f) const {
        A copy = transform(in, target_frame, timeout);
        tf2::convert(copy, out);
        return out;
    }

    // Transform, advanced api, with pre-allocation
    template <typename T>
    T& transform(const T& in, T& out,
                 const std::string& target_frame,  // NOLINT
                 const automsgs::msgs::builtin_interfaces::Time& target_time,
                 const std::string& fixed_frame, float timeout = 0.0f) const {
        // Convert tf2::Time (uint64_t nanoseconds) to
        // automsgs::msgs::builtin_interfaces::Time
        const tf2::Time tf2_time = tf2::getTimestamp(in);
        automsgs::msgs::builtin_interfaces::Time source_time;
        source_time.set_sec(static_cast<int32_t>(tf2_time / 1'000'000'000));
        source_time.set_nanosec(static_cast<uint32_t>(tf2_time % 1'000'000'000));
        // Get transform in commsgs format
        const auto& commsgs_transform =
            lookupTransform(target_frame, target_time, tf2::getFrameId(in),
                            source_time, fixed_frame, timeout);
        // Convert automsgs::msgs::geometry_msgs::TransformStamped to
        // geometry_msgs::TransformStamped
        geometry_msgs::TransformStamped tf2_transform;
        tf2_transform.header.stamp =
            static_cast<uint64_t>(commsgs_transform.header().stamp().sec()) *
                1000000000ULL +
            static_cast<uint64_t>(commsgs_transform.header().stamp().nanosec());
        tf2_transform.header.frame_id = commsgs_transform.header().frame_id();
        tf2_transform.child_frame_id = commsgs_transform.child_frame_id();
        tf2_transform.transform.translation.x =
            commsgs_transform.transform().translation().x();
        tf2_transform.transform.translation.y =
            commsgs_transform.transform().translation().y();
        tf2_transform.transform.translation.z =
            commsgs_transform.transform().translation().z();
        tf2_transform.transform.rotation.x =
            commsgs_transform.transform().rotation().x();
        tf2_transform.transform.rotation.y =
            commsgs_transform.transform().rotation().y();
        tf2_transform.transform.rotation.z =
            commsgs_transform.transform().rotation().z();
        tf2_transform.transform.rotation.w =
            commsgs_transform.transform().rotation().w();
        // do the transform
        tf2::doTransform(in, out, tf2_transform);
        return out;
    }

    // transform, advanced api, no pre-allocation
    template <typename T>
    T transform(const T& in, const std::string& target_frame,
                const automsgs::msgs::builtin_interfaces::Time& target_time,
                const std::string& fixed_frame, float timeout = 0.0f) const {
        T out;
        return transform(in, out, target_frame, target_time, fixed_frame,
                         timeout);
    }

    // Transform, advanced api, different types, with pre-allocation
    template <typename A, typename B>
    B& transform(const A& in, B& out,
                 const std::string& target_frame,  // NOLINT
                 const automsgs::msgs::builtin_interfaces::Time& target_time,
                 const std::string& fixed_frame, float timeout = 0.0f) const {
        // do the transform
        A copy = transform(in, target_frame, target_time, fixed_frame, timeout);
        tf2::convert(copy, out);
        return out;
    }
};  // class

}  // namespace transform
}  // namespace autonomy