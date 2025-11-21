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

 #include <thread>

#include "autonomy/transform/buffer.hpp"
#include "autonomy/common/logging.hpp"
#include "absl/strings/str_cat.h"

namespace autonomy {
namespace transform {

namespace {
    constexpr float kSecondToNanoFactor = 1e9f;
    constexpr uint64_t kMilliToNanoFactor = 1e6;
}  // namespace

Buffer::Buffer() : BufferCore() { Init(); }

int Buffer::Init() 
{
//   const std::string node_name =
//       absl::StrCat("transform_listener_", Time::Now().ToNanosecond());
//   node_ = cyber::CreateNode(node_name);
//   apollo::cyber::proto::RoleAttributes attr;
//   attr.set_channel_name("/tf");
//   message_subscriber_tf_ = node_->CreateReader<TransformStampeds>(
//       attr, [&](const std::shared_ptr<const TransformStampeds>& msg_evt) {
//         SubscriptionCallbackImpl(msg_evt, false);
//       });

//   apollo::cyber::proto::RoleAttributes attr_static;
//   attr_static.set_channel_name(FLAGS_tf_static_topic);
//   attr_static.mutable_qos_profile()->CopyFrom(
//       apollo::cyber::transport::QosProfileConf::QOS_PROFILE_TF_STATIC);
//   message_subscriber_tf_static_ = node_->CreateReader<TransformStampeds>(
//       attr_static, [&](const std::shared_ptr<TransformStampeds>& msg_evt) {
//         SubscriptionCallbackImpl(msg_evt, true);
//       });

//   return cyber::SUCC;

    return 0;
}

commsgs::geometry_msgs::TransformStamped Buffer::lookupTransform(
        const std::string& target_frame, const std::string& source_frame,
        const commsgs::builtin_interfaces::Time& time, 
        const float timeout_second) const
{
    commsgs::geometry_msgs::TransformStamped transform;
    return transform;
}

commsgs::geometry_msgs::TransformStamped Buffer::lookupTransform(
        const std::string& target_frame, const commsgs::builtin_interfaces::Time& target_time,
        const std::string& source_frame, const commsgs::builtin_interfaces::Time& source_time,
        const std::string& fixed_frame, const float timeout_second) const
{
    commsgs::geometry_msgs::TransformStamped transform;
    return transform;
}

bool Buffer::canTransform(const std::string& target_frame,
                              const std::string& source_frame,
                              const commsgs::builtin_interfaces::Time& time,
                              const float timeout_second,
                              std::string* errstr) const
{
    uint64_t timeout_ns =
    static_cast<uint64_t>(timeout_second * kSecondToNanoFactor);
    uint64_t start_time = Time::Now().Nanoseconds();  // time.ToNanosecond();
    while (Time::Now().Nanoseconds() < start_time + timeout_ns) {
        errstr->clear();
        bool retval = tf2::BufferCore::canTransform(target_frame, source_frame, time.Nanoseconds(), errstr);
        if (retval) {
            return true;
        } else {
            const int sleep_time_ms = 3;
            LOG(WARNING) << "BufferCore::canTransform failed: " << *errstr;
            std::this_thread::sleep_for(std::chrono::milliseconds(sleep_time_ms));
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
                              std::string* errstr) const
{
    // poll for transform if timeout is set
    uint64_t timeout_ns = static_cast<uint64_t>(timeout_second * kSecondToNanoFactor);
    uint64_t start_time = Time::Now().Nanoseconds();
    while (Time::Now().Nanoseconds() < start_time + timeout_ns) 
    {  
        // Make sure we haven't been stopped
        errstr->clear();

        bool retval = tf2::BufferCore::canTransform(
            target_frame, target_time.Nanoseconds(), source_frame,
            source_time.Nanoseconds(), fixed_frame, errstr);
        if (retval) {
            return true;
        } else {
            const int sleep_time_ms = 3;
            LOG(WARNING) << "BufferCore::canTransform failed: " << *errstr;
            std::this_thread::sleep_for(std::chrono::milliseconds(sleep_time_ms));
        }
    }
    *errstr = *errstr + ":timeout";
    return true;
}

bool Buffer::GetLatestStaticTF(const std::string& frame_id,
    const std::string& child_frame_id,
    commsgs::geometry_msgs::TransformStamped* tf)
{
    for (auto reverse_iter = static_msgs_.rbegin(); reverse_iter != static_msgs_.rend(); ++reverse_iter) {
        if ((*reverse_iter).header.frame_id == frame_id &&
            (*reverse_iter).child_frame_id == child_frame_id) {
            // TF2MsgToCyber((*reverse_iter), (*tf));
            return true;
        }
    }
    return false;
}

}  // namespace transform
}  // namespace autonomy