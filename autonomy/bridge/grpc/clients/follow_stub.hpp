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
#include <automsgs/task/tracker.pb.h>

namespace autonomy {
namespace bridge {
namespace grpc {
namespace clients {

/**
 * @brief Traits for Follow goal/feedback channels.
 * @see GoalChannelStub
 */
struct FollowTraits {
    using Goal = ::autonomy::task::proto::TrackerGoal;
    using Feedback = ::autonomy::task::proto::TrackerFeedback;
    using Request = proto::FollowCommandRequest;
    using Response = proto::FollowCommandResponse;

    static constexpr proto::TaskType kTaskType = proto::TASK_TYPE_FOLLOW;
    static constexpr const char* kGoalChannel =
        ::autonomy::task::kTrackingGoal;
    static constexpr const char* kFeedbackChannel =
        ::autonomy::task::kTrackingFeedback;

    /** @brief Convert a bridge follow request into a tracker goal. */
    static Goal ConvertToGoal(const Request& request);

    /** @brief Convert tracker feedback into a bridge follow response. */
    static Response ConvertFromFeedback(const Feedback& feedback,
                                        const Request& last);

    /** @brief Build an immediate ack-style follow response. */
    static Response MakeResponse(const Request& request, bool success,
                                 bool final, const std::string& message);

    /** @brief Check whether feedback reports a terminal tracker status. */
    static bool CheckTerminalStatus(const Feedback& feedback);
};

/**
 * @brief Bridge stub that forwards Follow commands to the tracking task.
 */
class FollowStub
{
public:
    using StreamCallback =
        std::function<void(const proto::FollowCommandResponse& response)>;

    AUTONOMY_SMART_PTR_DEFINITIONS(FollowStub)

    /**
     * @brief Construct the follow channel session.
     * @param[in] node Autolink node.
     * @param[in] muxer Shared task muxer.
     */
    FollowStub(std::shared_ptr<autolink::Node> node,
               std::shared_ptr<TaskMuxer> muxer);

    /**
     * @brief Handle a follow command and stream status updates.
     * @param[in] request Follow command request.
     * @param[in] stream_callback Stream sink.
     * @return false if rejected immediately.
     */
    bool HandleCommand(const proto::FollowCommandRequest& request,
                       StreamCallback stream_callback);

    /** @brief Cancel the active follow session. */
    void CancelActiveSession();

private:
    GoalChannelStub<FollowTraits> channel_;
};

}  // namespace clients
}  // namespace grpc
}  // namespace bridge
}  // namespace autonomy
