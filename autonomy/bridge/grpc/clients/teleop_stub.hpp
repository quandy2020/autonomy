/*
 * Copyright 2026 The Openbot Authors
 */

#pragma once

#include <functional>
#include <memory>
#include <string>

#include "autolink/node/node.hpp"
#include "autonomy/bridge/grpc/clients/goal_channel_stub.hpp"
#include "autonomy/bridge/grpc/task_muxer.hpp"
#include "autonomy/bridge/proto/external_command_service.pb.h"
#include "autonomy/common/macros.hpp"
#include "autonomy/task/common/names.hpp"
#include <automsgs/task/teleop.pb.h>

namespace autonomy {
namespace bridge {
namespace grpc {
namespace clients {

/**
 * @brief Traits for Teleop goal/feedback channels.
 * @see GoalChannelStub
 */
struct TeleopTraits {
    using Goal = ::autonomy::task::proto::TeleopGoal;
    using Feedback = ::autonomy::task::proto::TeleopFeedback;
    using Request = proto::TeleopCommandRequest;
    using Response = proto::TeleopCommandResponse;

    static constexpr proto::TaskType kTaskType = proto::TASK_TYPE_TELEOP;
    static constexpr const char* kGoalChannel = ::autonomy::task::kTeleopGoal;
    static constexpr const char* kFeedbackChannel =
        ::autonomy::task::kTeleopFeedback;

    /** @brief Convert a bridge teleop request into a teleop goal. */
    static Goal ConvertToGoal(const Request& request);

    /** @brief Convert teleop feedback into a bridge teleop response. */
    static Response ConvertFromFeedback(const Feedback& feedback,
                                        const Request& last);

    /** @brief Build an immediate ack-style teleop response. */
    static Response MakeResponse(const Request& request, bool success,
                                 bool final, const std::string& message);

    /** @brief Check whether feedback reports a terminal teleop status. */
    static bool CheckTerminalStatus(const Feedback& feedback);

    /**
     * @brief Check whether feedback should be forwarded to the stream.
     * @param[in] feedback Task feedback.
     * @param[in] session_active Whether START has activated the session.
     */
    static bool CheckEmitFeedback(const Feedback& feedback,
                                  bool session_active);
};

/**
 * @brief Bridge stub that publishes teleop commands via GoalChannelStub.
 *
 * START / VELOCITY use optimistic acks; REJECTED / TIMEOUT / IDLE arrive from
 * feedback.
 */
class TeleopStub
{
public:
    using StreamCallback =
        std::function<void(const proto::TeleopCommandResponse& response)>;

    AUTONOMY_SMART_PTR_DEFINITIONS(TeleopStub)

    /**
     * @brief Construct the teleop channel session.
     * @param[in] node Autolink node.
     * @param[in] muxer Shared task muxer (may be null).
     */
    explicit TeleopStub(std::shared_ptr<autolink::Node> node,
                        std::shared_ptr<TaskMuxer> muxer = nullptr);

    /**
     * @brief Handle a teleop command and stream status updates.
     * @param[in] request Teleop command request.
     * @param[in] stream_callback Stream sink.
     * @return false if rejected immediately.
     */
    bool HandleCommand(const proto::TeleopCommandRequest& request,
                       StreamCallback stream_callback);

    /** @brief Stop the active session and release the muxer slot. */
    void ResetSession();

private:
    GoalChannelStub<TeleopTraits> channel_;
};

}  // namespace clients
}  // namespace grpc
}  // namespace bridge
}  // namespace autonomy
