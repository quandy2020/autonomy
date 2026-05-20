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

#include "autonomy/transform/buffer.hpp"

#include <thread>

#include "autonomy/common/logging.hpp"
#include "autonomy/transform/tf2/exceptions.h"

namespace autonomy {
namespace transform {

namespace {
constexpr float kSecondToNanoFactor = 1e9f;
constexpr uint64_t kMilliToNanoFactor = 1e6;

uint64_t ToTf2TimeNs(const commsgs::builtin_interfaces::Time& time) {
    return time.ToUnixTimeNanos();
}

bool IsFutureExtrapolation(const std::string& err) {
    return err.find("extrapolation into the future") != std::string::npos;
}
}  // namespace

Buffer::Buffer() : BufferCore() {
    Init();
}

int Buffer::Init() {
    return 0;
}

void Buffer::clear() {
    static_msgs_.clear();
    tf2::BufferCore::clear();
}

commsgs::geometry_msgs::TransformStamped Buffer::lookupTransform(
    const std::string& target_frame, const std::string& source_frame,
    const commsgs::builtin_interfaces::Time& time,
    const float timeout_second) const {
    // Fast path: identity transform
    if (target_frame == source_frame) {
        commsgs::geometry_msgs::TransformStamped out;
        out.header.stamp = time;
        out.header.frame_id = target_frame;
        out.child_frame_id = source_frame;
        out.transform.translation.x = 0.0;
        out.transform.translation.y = 0.0;
        out.transform.translation.z = 0.0;
        out.transform.rotation.x = 0.0;
        out.transform.rotation.y = 0.0;
        out.transform.rotation.z = 0.0;
        out.transform.rotation.w = 1.0;
        return out;
    }

    std::string err;
    if (!const_cast<Buffer*>(this)->canTransform(target_frame, source_frame,
                                                 time, timeout_second, &err)) {
        throw tf2::TimeoutException("TF lookupTransform timeout: " + err);
    }

    uint64_t tf2_time_ns = ToTf2TimeNs(time);
    if (tf2_time_ns != 0 && IsFutureExtrapolation(err)) {
        tf2_time_ns = 0ULL;
    }
    const auto tf2_transform =
        tf2::BufferCore::lookupTransform(target_frame, source_frame, tf2_time_ns);
    commsgs::geometry_msgs::TransformStamped out;
    TF2MsgToConvert(tf2_transform, out);
    return out;
}

commsgs::geometry_msgs::TransformStamped Buffer::lookupTransform(
    const std::string& target_frame,
    const commsgs::builtin_interfaces::Time& target_time,
    const std::string& source_frame,
    const commsgs::builtin_interfaces::Time& source_time,
    const std::string& fixed_frame, const float timeout_second) const {
    // Fast path: identity transform
    if (target_frame == source_frame) {
        commsgs::geometry_msgs::TransformStamped out;
        out.header.stamp = target_time;
        out.header.frame_id = target_frame;
        out.child_frame_id = source_frame;
        out.transform.translation.x = 0.0;
        out.transform.translation.y = 0.0;
        out.transform.translation.z = 0.0;
        out.transform.rotation.x = 0.0;
        out.transform.rotation.y = 0.0;
        out.transform.rotation.z = 0.0;
        out.transform.rotation.w = 1.0;
        return out;
    }

    std::string err;
    if (!const_cast<Buffer*>(this)->canTransform(
            target_frame, target_time, source_frame, source_time, fixed_frame,
            timeout_second, &err)) {
        throw tf2::TimeoutException("TF lookupTransform timeout: " + err);
    }

    uint64_t target_ns = ToTf2TimeNs(target_time);
    uint64_t source_ns = ToTf2TimeNs(source_time);
    if ((target_ns != 0 || source_ns != 0) && IsFutureExtrapolation(err)) {
        target_ns = 0ULL;
        source_ns = 0ULL;
    }
    const auto tf2_transform = tf2::BufferCore::lookupTransform(
        target_frame, target_ns, source_frame, source_ns, fixed_frame);
    commsgs::geometry_msgs::TransformStamped out;
    TF2MsgToConvert(tf2_transform, out);
    return out;
}

bool Buffer::canTransform(const std::string& target_frame,
                          const std::string& source_frame,
                          const commsgs::builtin_interfaces::Time& time,
                          const float timeout_second,
                          std::string* errstr) const {
    const uint64_t requested_ns = ToTf2TimeNs(time);
    uint64_t timeout_ns =
        static_cast<uint64_t>(timeout_second * kSecondToNanoFactor);
    const uint64_t start_time = ToTf2TimeNs(Time::Now());
    while (ToTf2TimeNs(Time::Now()) < start_time + timeout_ns) {
        errstr->clear();
        bool retval = tf2::BufferCore::canTransform(
            target_frame, source_frame, requested_ns, errstr);
        if (retval) {
            return true;
        }
        // In single-process mock mode, cmd_vel integration may publish TF
        // slightly behind the caller thread. If caller asks for a tiny future
        // timestamp, gracefully fall back to latest available transform.
        if (requested_ns != 0 && IsFutureExtrapolation(*errstr)) {
            std::string latest_err;
            if (tf2::BufferCore::canTransform(target_frame, source_frame, 0ULL,
                                              &latest_err)) {
                return true;
            }
        }
        {
            const int sleep_time_ms = 3;
            LOG(WARNING) << "BufferCore::canTransform failed: " << *errstr;
            std::this_thread::sleep_for(
                std::chrono::milliseconds(sleep_time_ms));
        }
    }
    *errstr = *errstr + ":timeout";
    return false;
}

bool Buffer::canTransform(const std::string& target_frame,
                          const commsgs::builtin_interfaces::Time& target_time,
                          const std::string& source_frame,
                          const commsgs::builtin_interfaces::Time& source_time,
                          const std::string& fixed_frame,
                          const float timeout_second,
                          std::string* errstr) const {
    const uint64_t target_ns = ToTf2TimeNs(target_time);
    const uint64_t source_ns = ToTf2TimeNs(source_time);
    // poll for transform if timeout is set
    uint64_t timeout_ns =
        static_cast<uint64_t>(timeout_second * kSecondToNanoFactor);
    const uint64_t start_time = ToTf2TimeNs(Time::Now());
    while (ToTf2TimeNs(Time::Now()) < start_time + timeout_ns) {
        // Make sure we haven't been stopped
        errstr->clear();

        bool retval = tf2::BufferCore::canTransform(
            target_frame, target_ns, source_frame, source_ns, fixed_frame,
            errstr);
        if (retval) {
            return true;
        }
        if ((target_ns != 0 || source_ns != 0) &&
            IsFutureExtrapolation(*errstr)) {
            std::string latest_err;
            if (tf2::BufferCore::canTransform(target_frame, 0ULL, source_frame,
                                              0ULL, fixed_frame,
                                              &latest_err)) {
                return true;
            }
        }
        {
            const int sleep_time_ms = 3;
            LOG(WARNING) << "BufferCore::canTransform failed: " << *errstr;
            std::this_thread::sleep_for(
                std::chrono::milliseconds(sleep_time_ms));
        }
    }
    *errstr = *errstr + ":timeout";
    return false;
}

bool Buffer::GetLatestStaticTF(const std::string& frame_id,
                               const std::string& child_frame_id,
                               commsgs::geometry_msgs::TransformStamped* tf) {
    for (auto reverse_iter = static_msgs_.rbegin();
         reverse_iter != static_msgs_.rend(); ++reverse_iter) {
        if ((*reverse_iter).header.frame_id == frame_id &&
            (*reverse_iter).child_frame_id == child_frame_id) {
            TF2MsgToConvert((*reverse_iter), (*tf));
            return true;
        }
    }
    return false;
}

void Buffer::SubscriptionCallback(
    const std::shared_ptr<const commsgs::geometry_msgs::TransformStampeds>&
        msg_evt) {
    SubscriptionCallbackImpl(msg_evt, false);
}

void Buffer::StaticSubscriptionCallback(
    const std::shared_ptr<const commsgs::geometry_msgs::TransformStampeds>&
        msg_evt) {
    SubscriptionCallbackImpl(msg_evt, true);
}

void Buffer::SubscriptionCallbackImpl(
    const std::shared_ptr<const commsgs::geometry_msgs::TransformStampeds>&
        msg_evt,
    bool is_static) {
    commsgs::builtin_interfaces::Time now = Time::Now();
    std::string authority =
        "autolink_tf";  // msg_evt.getPublisherName(); // lookup the authority
    if (now.ToUnixTimeNanos() < last_update_.ToUnixTimeNanos()) {
        AINFO << "Detected jump back in time. Clearing TF buffer.";
        clear();
        // cache static transform stamped again.
        for (const auto& geo_msg : static_msgs_) {
            setTransform(geo_msg, authority, true);
        }
    }
    last_update_ = now;

    for (size_t i = 0; i < msg_evt->transforms.size(); i++) {
        try {
            const auto& trans = msg_evt->transforms[i];

            // Convert to geometry_msgs::TransformStamped for tf2
            geometry_msgs::TransformStamped geo_msg;
            // Convert timestamp: sec * 1e9 + nanosec
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

            if (is_static) {
                static_msgs_.push_back(geo_msg);
            }
            setTransform(geo_msg, authority, is_static);
        } catch (tf2::TransformException& ex) {
            std::string temp = ex.what();
            AERROR << "Failure to set received transform:" << temp.c_str();
        }
    }
}

void Buffer::TF2MsgToConvert(
    const geometry_msgs::TransformStamped& tf2_trans_stamped,
    commsgs::geometry_msgs::TransformStamped& trans_stamped) const {
    // Convert from geometry_msgs (tf2 internal) to commsgs::geometry_msgs
    // (autolink) header
    trans_stamped.header.stamp.sec =
        static_cast<int32_t>(tf2_trans_stamped.header.stamp / 1000000000ULL);
    trans_stamped.header.stamp.nanosec =
        static_cast<uint32_t>(tf2_trans_stamped.header.stamp % 1000000000ULL);
    trans_stamped.header.frame_id = tf2_trans_stamped.header.frame_id;

    // child_frame_id
    trans_stamped.child_frame_id = tf2_trans_stamped.child_frame_id;

    // translation
    trans_stamped.transform.translation.x =
        tf2_trans_stamped.transform.translation.x;
    trans_stamped.transform.translation.y =
        tf2_trans_stamped.transform.translation.y;
    trans_stamped.transform.translation.z =
        tf2_trans_stamped.transform.translation.z;

    // rotation
    trans_stamped.transform.rotation.x = tf2_trans_stamped.transform.rotation.x;
    trans_stamped.transform.rotation.y = tf2_trans_stamped.transform.rotation.y;
    trans_stamped.transform.rotation.z = tf2_trans_stamped.transform.rotation.z;
    trans_stamped.transform.rotation.w = tf2_trans_stamped.transform.rotation.w;
}

}  // namespace transform
}  // namespace autonomy