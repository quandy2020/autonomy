/**
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

#include <climits>
#include <cstddef>
#include <cstring>
#include <memory>
#include <string>
#include <vector>

#include "autolink/action/types.hpp"
#include "autolink/message/message_traits.hpp"
#include "autolink/message/protobuf_factory.hpp"
#include "autolink/proto/action.pb.h"

namespace autolink {
namespace action {
namespace internal {

// SendGoal service request - wrapper around protobuf with conversion
template <typename ActionT>
class SendGoalRequest
{
public:
    SendGoalRequest() {}

    // Public members for direct access (compatible with struct usage)
    GoalUUID goal_id;
    typename ActionT::Goal goal;

    // Convert GoalUUID to proto::GoalUUID
    static proto::GoalUUID GoalUUIDToProto(const GoalUUID& uuid) {
        proto::GoalUUID proto_uuid;
        proto_uuid.set_uuid(std::string(
            reinterpret_cast<const char*>(uuid.data()), uuid.size()));
        return proto_uuid;
    }

    // Convert proto::GoalUUID to GoalUUID
    static GoalUUID ProtoToGoalUUID(const proto::GoalUUID& proto_uuid) {
        GoalUUID uuid;
        if (proto_uuid.uuid().size() == 16) {
            const std::string& uuid_str = proto_uuid.uuid();
            std::copy(uuid_str.begin(), uuid_str.end(), uuid.begin());
        }
        return uuid;
    }

    // Convert to protobuf message
    proto::SendGoalRequest ToProto() const {
        proto::SendGoalRequest proto_msg;
        *proto_msg.mutable_goal_id() = GoalUUIDToProto(goal_id);

        std::string goal_str;
        if (message::SerializeToString(goal, &goal_str)) {
            proto_msg.set_goal(goal_str);
        }
        return proto_msg;
    }

    // Convert from protobuf message
    void FromProto(const proto::SendGoalRequest& proto_msg) {
        goal_id = ProtoToGoalUUID(proto_msg.goal_id());

        if (!proto_msg.goal().empty()) {
            message::ParseFromString(proto_msg.goal(), &goal);
        }
    }

    // Serialization methods required by Autolink
    bool SerializeToString(std::string* output) const {
        if (!output)
            return false;
        proto::SendGoalRequest proto_msg = ToProto();
        return proto_msg.SerializeToString(output);
    }

    bool ParseFromString(const std::string& input) {
        proto::SendGoalRequest proto_msg;
        if (!proto_msg.ParseFromString(input)) {
            return false;
        }
        FromProto(proto_msg);
        return true;
    }
};

// SendGoal service response - wrapper around protobuf
class SendGoalResponse
{
public:
    SendGoalResponse() : accepted(false), stamp(0) {}

    proto::SendGoalResponse ToProto() const {
        proto::SendGoalResponse proto_msg;
        proto_msg.set_accepted(accepted);
        proto_msg.set_stamp_sec(stamp / 1000000000LL);
        proto_msg.set_stamp_nanosec(
            static_cast<uint32_t>(stamp % 1000000000LL));
        return proto_msg;
    }

    void FromProto(const proto::SendGoalResponse& proto_msg) {
        accepted = proto_msg.accepted();
        stamp = static_cast<int64_t>(proto_msg.stamp_sec()) * 1000000000LL +
                proto_msg.stamp_nanosec();
    }

    bool SerializeToString(std::string* output) const {
        if (!output)
            return false;
        proto::SendGoalResponse proto_msg = ToProto();
        return proto_msg.SerializeToString(output);
    }

    bool ParseFromString(const std::string& input) {
        proto::SendGoalResponse proto_msg;
        if (!proto_msg.ParseFromString(input)) {
            return false;
        }
        FromProto(proto_msg);
        return true;
    }

    // Public members for direct access (compatible with struct usage)
    bool accepted;
    int64_t stamp;
};

// CancelGoal service request - use protobuf message directly
// But we need to add conversion methods
class CancelGoalRequest
{
public:
    CancelGoalRequest() {}

    // Public member for direct access
    GoalUUID goal_id;

    proto::CancelGoalRequest ToProto() const {
        proto::CancelGoalRequest proto_msg;
        proto::GoalUUID* uuid_msg = proto_msg.mutable_goal_id();
        uuid_msg->set_uuid(std::string(
            reinterpret_cast<const char*>(goal_id.data()), goal_id.size()));
        return proto_msg;
    }

    void FromProto(const proto::CancelGoalRequest& proto_msg) {
        if (proto_msg.has_goal_id() &&
            proto_msg.goal_id().uuid().size() == 16) {
            const std::string& uuid_str = proto_msg.goal_id().uuid();
            std::copy(uuid_str.begin(), uuid_str.end(), goal_id.begin());
        }
    }

    bool SerializeToString(std::string* output) const {
        if (!output)
            return false;
        proto::CancelGoalRequest proto_msg = ToProto();
        return proto_msg.SerializeToString(output);
    }

    bool ParseFromString(const std::string& input) {
        proto::CancelGoalRequest proto_msg;
        if (!proto_msg.ParseFromString(input)) {
            return false;
        }
        FromProto(proto_msg);
        return true;
    }
};

// CancelGoal service response - wrapper
class CancelGoalResponse
{
public:
    CancelGoalResponse() : goals_canceling(0) {}

    proto::CancelGoalResponse ToProto() const {
        proto::CancelGoalResponse proto_msg;
        proto_msg.set_return_code(goals_canceling > 0 ? 0
                                                      : 1);  // 0 = ERROR_NONE
        if (goals_canceling > 0) {
            // Add goal_id to goals_canceling list if needed
        }
        return proto_msg;
    }

    void FromProto(const proto::CancelGoalResponse& proto_msg) {
        goals_canceling =
            proto_msg.return_code() == 0 ? proto_msg.goals_canceling_size() : 0;
    }

    bool SerializeToString(std::string* output) const {
        if (!output)
            return false;
        proto::CancelGoalResponse proto_msg = ToProto();
        return proto_msg.SerializeToString(output);
    }

    bool ParseFromString(const std::string& input) {
        proto::CancelGoalResponse proto_msg;
        if (!proto_msg.ParseFromString(input)) {
            return false;
        }
        FromProto(proto_msg);
        return true;
    }

    // Public member for direct access
    int32_t goals_canceling;
};

// GetResult service request - wrapper
class GetResultRequest
{
public:
    GetResultRequest() {}

    // Public member for direct access
    GoalUUID goal_id;

    proto::GetResultRequest ToProto() const {
        proto::GetResultRequest proto_msg;
        proto::GoalUUID* uuid_msg = proto_msg.mutable_goal_id();
        uuid_msg->set_uuid(std::string(
            reinterpret_cast<const char*>(goal_id.data()), goal_id.size()));
        return proto_msg;
    }

    void FromProto(const proto::GetResultRequest& proto_msg) {
        if (proto_msg.has_goal_id() &&
            proto_msg.goal_id().uuid().size() == 16) {
            const std::string& uuid_str = proto_msg.goal_id().uuid();
            std::copy(uuid_str.begin(), uuid_str.end(), goal_id.begin());
        }
    }

    bool SerializeToString(std::string* output) const {
        if (!output)
            return false;
        proto::GetResultRequest proto_msg = ToProto();
        return proto_msg.SerializeToString(output);
    }

    bool ParseFromString(const std::string& input) {
        proto::GetResultRequest proto_msg;
        if (!proto_msg.ParseFromString(input)) {
            return false;
        }
        FromProto(proto_msg);
        return true;
    }
};

// GetResult service response - wrapper
template <typename ActionT>
class GetResultResponse
{
public:
    GetResultResponse() : status(0) {}

    // Public members for direct access
    int8_t status;
    typename ActionT::Result result;

    proto::GetResultResponse ToProto() const {
        proto::GetResultResponse proto_msg;
        proto_msg.set_status(status);
        std::string result_str;
        if (message::SerializeToString(result, &result_str)) {
            proto_msg.set_result(result_str);
        }
        return proto_msg;
    }

    void FromProto(const proto::GetResultResponse& proto_msg) {
        status = static_cast<int8_t>(proto_msg.status());
        if (!proto_msg.result().empty()) {
            message::ParseFromString(proto_msg.result(), &result);
        }
    }

    bool SerializeToString(std::string* output) const {
        if (!output)
            return false;
        proto::GetResultResponse proto_msg = ToProto();
        return proto_msg.SerializeToString(output);
    }

    bool ParseFromString(const std::string& input) {
        proto::GetResultResponse proto_msg;
        if (!proto_msg.ParseFromString(input)) {
            return false;
        }
        FromProto(proto_msg);
        return true;
    }
};

// Feedback message - wrapper
template <typename ActionT>
class FeedbackMessage
{
public:
    FeedbackMessage() {}

    // Public members for direct access
    GoalUUID goal_id;
    typename ActionT::Feedback feedback;

    proto::FeedbackMessage ToProto() const {
        proto::FeedbackMessage proto_msg;
        proto::GoalUUID* uuid_msg = proto_msg.mutable_goal_id();
        uuid_msg->set_uuid(std::string(
            reinterpret_cast<const char*>(goal_id.data()), goal_id.size()));

        std::string feedback_str;
        if (message::SerializeToString(feedback, &feedback_str)) {
            proto_msg.set_feedback(feedback_str);
        }
        return proto_msg;
    }

    void FromProto(const proto::FeedbackMessage& proto_msg) {
        if (proto_msg.has_goal_id() &&
            proto_msg.goal_id().uuid().size() == 16) {
            const std::string& uuid_str = proto_msg.goal_id().uuid();
            std::copy(uuid_str.begin(), uuid_str.end(), goal_id.begin());
        }

        if (!proto_msg.feedback().empty()) {
            message::ParseFromString(proto_msg.feedback(), &feedback);
        }
    }

    bool SerializeToString(std::string* output) const {
        if (!output)
            return false;
        proto::FeedbackMessage proto_msg = ToProto();
        return proto_msg.SerializeToString(output);
    }

    bool ParseFromString(const std::string& input) {
        proto::FeedbackMessage proto_msg;
        if (!proto_msg.ParseFromString(input)) {
            return false;
        }
        FromProto(proto_msg);
        return true;
    }

    // SerializeToArray and ParseFromArray for Autolink message transport
    bool SerializeToArray(void* data, int size) const {
        if (!data || size <= 0)
            return false;
        std::string serialized;
        if (!SerializeToString(&serialized)) {
            return false;
        }
        if (static_cast<int>(serialized.size()) > size) {
            return false;
        }
        memcpy(data, serialized.data(), serialized.size());
        return true;
    }

    bool ParseFromArray(const void* data, int size) {
        if (!data || size <= 0)
            return false;
        std::string input(reinterpret_cast<const char*>(data), size);
        return ParseFromString(input);
    }

    /// SHM / transmitter uses \ref autolink::message::ByteSize, which selects
    /// types exposing \c ByteSizeLong() (protobuf convention). Without this,
    /// \c ByteSize returns -1, which becomes \c SIZE_MAX and breaks shared
    /// memory serialization.
    std::size_t ByteSizeLong() const {
        std::string serialized;
        if (!SerializeToString(&serialized)) {
            return 0;
        }
        return serialized.size();
    }

    int ByteSize() const {
        const std::size_t n = ByteSizeLong();
        if (n > static_cast<std::size_t>(INT_MAX)) {
            return INT_MAX;
        }
        return static_cast<int>(n);
    }

    // Add TypeName and GetDescriptorString for Autolink message registration
    static std::string TypeName() {
        return proto::FeedbackMessage::descriptor()->full_name();
    }

    static void GetDescriptorString(const std::string& type,
                                    std::string* desc_str) {
        if (!desc_str)
            return;
        // Use ProtobufFactory to get descriptor string, similar to RawMessage
        message::ProtobufFactory::Instance()->GetDescriptorString(
            proto::FeedbackMessage::descriptor()->full_name(), desc_str);
    }
};

// Status message - simplified version for internal use
struct GoalStatusInfo {
    GoalUUID goal_id;
    int8_t status;
};

struct StatusMessage {
    std::vector<GoalStatusInfo> status_list;

    // Add serialization methods for compatibility
    bool SerializeToString(std::string* output) const {
        // StatusMessage is mainly for internal use via Channel
        // Channel messages need serialization, but this is complex
        // For now, return false to indicate it's not directly serializable
        (void)output;
        return false;
    }

    bool ParseFromString(const std::string& input) {
        (void)input;
        return false;
    }
};

}  // namespace internal
}  // namespace action
}  // namespace autolink
