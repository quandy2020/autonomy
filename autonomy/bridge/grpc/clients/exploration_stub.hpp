/*
 * Copyright 2026 The Openbot Authors
 */

#pragma once

#include <functional>
#include <memory>
#include <mutex>
#include <string>

#include "autolink/node/node.hpp"
#include "autonomy/bridge/grpc/clients/stream_session.hpp"
#include "autonomy/bridge/grpc/task_muxer.hpp"
#include "autonomy/bridge/proto/external_command_service.pb.h"
#include "autonomy/common/macros.hpp"
#include <automsgs/msgs/geometry_msgs/pose_stamped.pb.h>
#include <automsgs/msgs/std_msgs/bool.pb.h>
#include <automsgs/task/mapping.pb.h>

namespace autonomy {
namespace bridge {
namespace grpc {
namespace clients {

/**
 * @brief Exploration bridge over mapping goal, waypoint, and finished channels.
 *
 * Uses @ref StreamSessionState for callback/session bookkeeping. Not a single
 * `GoalChannelStub` because exploration spans multiple topics.
 */
class ExplorationStub
{
public:
    using StreamCallback = std::function<void(
        const proto::ExplorationCommandResponse& response)>;

    AUTONOMY_SMART_PTR_DEFINITIONS(ExplorationStub)

    /**
     * @brief Construct exploration writers/readers.
     * @param[in] node Autolink node.
     * @param[in] muxer Shared task muxer.
     */
    ExplorationStub(std::shared_ptr<autolink::Node> node,
                    std::shared_ptr<TaskMuxer> muxer);

    /**
     * @brief Handle an exploration command and stream status updates.
     * @param[in] request Exploration command request.
     * @param[in] stream_callback Stream sink.
     * @return false if rejected immediately.
     */
    bool HandleCommand(const proto::ExplorationCommandRequest& request,
                       StreamCallback stream_callback);

    /** @brief Cancel the active exploration session. */
    void CancelActiveSession();

    /** @brief Return the latest exploration status snapshot. */
    proto::ExplorationCommandResponse GetSnapshot() const;

private:
    using Session =
        StreamSessionState<proto::ExplorationCommandRequest,
                           proto::ExplorationCommandResponse>;

    void HandleFinished(
        const std::shared_ptr<::automsgs::msgs::std_msgs::Bool>& message);
    void HandleMappingFeedback(
        const std::shared_ptr<::autonomy::task::proto::MappingFeedback>&
            feedback);
    bool PublishMappingGoal(const ::autonomy::task::proto::MappingGoal& goal);
    proto::ExplorationCommandResponse MakeResponse(
        const proto::ExplorationCommandRequest& request,
        proto::ExplorationStatus status, bool success, bool final,
        const std::string& message = "") const;

    std::shared_ptr<autolink::Node> node_;
    std::shared_ptr<TaskMuxer> muxer_;
    std::shared_ptr<autolink::Writer<::autonomy::task::proto::MappingGoal>>
        mapping_writer_;
    std::shared_ptr<autolink::Writer<::automsgs::msgs::geometry_msgs::PoseStamped>>
        waypoint_writer_;
    std::shared_ptr<autolink::Reader<::automsgs::msgs::std_msgs::Bool>>
        finished_reader_;
    std::shared_ptr<autolink::Reader<::autonomy::task::proto::MappingFeedback>>
        mapping_feedback_reader_;

    Session session_;
    proto::ExplorationStatus status_{proto::EXPLORATION_STATUS_IDLE};
    float progress_{0.f};
    std::string map_name_;
};

}  // namespace clients
}  // namespace grpc
}  // namespace bridge
}  // namespace autonomy
