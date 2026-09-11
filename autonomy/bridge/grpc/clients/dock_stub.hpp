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
#include <automsgs/task/charging.pb.h>

namespace autonomy {
namespace bridge {
namespace grpc {
namespace clients {

/**
 * @brief Traits for Dock / charging goal/feedback channels.
 * @see GoalChannelStub
 */
struct DockTraits {
    using Goal = ::autonomy::task::proto::ChargingGoal;
    using Feedback = ::autonomy::task::proto::ChargingFeedback;
    using Request = proto::DockCommandRequest;
    using Response = proto::DockCommandResponse;

    static constexpr proto::TaskType kTaskType = proto::TASK_TYPE_DOCK;
    static constexpr const char* kGoalChannel =
        ::autonomy::task::kChargingGoal;
    static constexpr const char* kFeedbackChannel =
        ::autonomy::task::kChargingFeedback;

    /** @brief Convert a bridge dock request into a charging goal. */
    static Goal ConvertToGoal(const Request& request);

    /** @brief Convert charging feedback into a bridge dock response. */
    static Response ConvertFromFeedback(const Feedback& feedback,
                                        const Request& last);

    /** @brief Build an immediate ack-style dock response. */
    static Response MakeResponse(const Request& request, bool success,
                                 bool final, const std::string& message);

    /** @brief Check whether feedback reports a terminal dock status. */
    static bool CheckTerminalStatus(const Feedback& feedback);
};

/**
 * @brief Bridge stub that forwards Dock commands to the charging task.
 */
class DockStub
{
public:
    using StreamCallback =
        std::function<void(const proto::DockCommandResponse& response)>;

    AUTONOMY_SMART_PTR_DEFINITIONS(DockStub)

    /**
     * @brief Construct the dock channel session.
     * @param[in] node Autolink node.
     * @param[in] muxer Shared task muxer.
     */
    DockStub(std::shared_ptr<autolink::Node> node,
             std::shared_ptr<TaskMuxer> muxer);

    /**
     * @brief Handle a dock command and stream status updates.
     * @param[in] request Dock command request.
     * @param[in] stream_callback Stream sink.
     * @return false if rejected immediately.
     */
    bool HandleCommand(const proto::DockCommandRequest& request,
                       StreamCallback stream_callback);

    /** @brief Cancel the active dock session. */
    void CancelActiveSession();

private:
    GoalChannelStub<DockTraits> channel_;
};

}  // namespace clients
}  // namespace grpc
}  // namespace bridge
}  // namespace autonomy
