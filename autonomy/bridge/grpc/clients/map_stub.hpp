/*
 * Copyright 2026 The Openbot Authors
 */

#pragma once

#include <functional>
#include <memory>
#include <mutex>
#include <string>

#include "autolink/node/node.hpp"
#include "autonomy/bridge/grpc/clients/goal_channel_stub.hpp"
#include "autonomy/bridge/grpc/task_muxer.hpp"
#include "autonomy/bridge/proto/external_command_service.pb.h"
#include "autonomy/common/macros.hpp"
#include "autonomy/task/common/names.hpp"
#include <automsgs/task/mapping.pb.h>

namespace autonomy {
namespace bridge {
namespace grpc {
namespace clients {

/**
 * @brief Traits for Map goal/feedback channels.
 * @see GoalChannelStub
 */
struct MapTraits {
    using Goal = ::autonomy::task::proto::MappingGoal;
    using Feedback = ::autonomy::task::proto::MappingFeedback;
    using Request = proto::MapCommandRequest;
    using Response = proto::MapCommandResponse;

    static constexpr proto::TaskType kTaskType = proto::TASK_TYPE_MAP;
    static constexpr const char* kGoalChannel = ::autonomy::task::kMappingGoal;
    static constexpr const char* kFeedbackChannel =
        ::autonomy::task::kMappingFeedback;

    /**
     * @brief Optional map-name provider installed by MapStub.
     * @note Non-owning callback; cleared in MapStub destructor.
     */
    static std::function<std::string()> map_name_fn;

    /** @brief Convert a bridge map request into a mapping goal. */
    static Goal ConvertToGoal(const Request& request);

    /** @brief Convert mapping feedback into a bridge map response. */
    static Response ConvertFromFeedback(const Feedback& feedback,
                                        const Request& last);

    /** @brief Build an immediate ack-style map response. */
    static Response MakeResponse(const Request& request, bool success,
                                 bool final, const std::string& message);

    /** @brief Check whether feedback reports a terminal map status. */
    static bool CheckTerminalStatus(const Feedback& feedback);
};

/**
 * @brief Bridge stub that forwards Map commands to the mapping task.
 */
class MapStub
{
public:
    using StreamCallback =
        std::function<void(const proto::MapCommandResponse& response)>;

    AUTONOMY_SMART_PTR_DEFINITIONS(MapStub)

    /**
     * @brief Construct the map channel session.
     * @param[in] node Autolink node.
     * @param[in] muxer Shared task muxer.
     */
    MapStub(std::shared_ptr<autolink::Node> node,
            std::shared_ptr<TaskMuxer> muxer);
    ~MapStub();

    /**
     * @brief Handle a map command and stream status updates.
     * @param[in] request Map command request.
     * @param[in] stream_callback Stream sink.
     * @return false if rejected immediately.
     */
    bool HandleCommand(const proto::MapCommandRequest& request,
                       StreamCallback stream_callback);

    /** @brief Return the cached current map name. */
    std::string GetCurrentMapName() const;

private:
    mutable std::mutex map_name_mutex_;
    std::string current_map_name_;
    GoalChannelStub<MapTraits> channel_;
};

}  // namespace clients
}  // namespace grpc
}  // namespace bridge
}  // namespace autonomy
