/*
 * Copyright 2026 The Openbot Authors
 */

#pragma once

#include <functional>
#include <memory>
#include <mutex>
#include <string>

#include "autonomy/bridge/grpc/clients/dock_stub.hpp"
#include "autonomy/bridge/grpc/clients/exploration_stub.hpp"
#include "autonomy/bridge/grpc/clients/follow_stub.hpp"
#include "autonomy/bridge/grpc/clients/navigator_stub.hpp"
#include "autonomy/bridge/grpc/task_muxer.hpp"
#include "autonomy/bridge/proto/external_command_service.pb.h"
#include "autonomy/common/macros.hpp"

namespace autonomy {
namespace bridge {
namespace grpc {
namespace clients {

/**
 * @brief Routes structured voice intents to navigation / follow / dock / explore.
 */
class VoiceStub
{
public:
    using StreamCallback =
        std::function<void(const proto::VoiceCommandResponse& response)>;

    AUTONOMY_SMART_PTR_DEFINITIONS(VoiceStub)

    /**
     * @brief Construct a voice router over domain stubs.
     * @param[in] muxer Shared task muxer.
     * @param[in] navigator Non-owning navigator stub.
     * @param[in] follow Non-owning follow stub.
     * @param[in] dock Non-owning dock stub.
     * @param[in] exploration Non-owning exploration stub.
     */
    VoiceStub(std::shared_ptr<TaskMuxer> muxer,
              clients::NavigatorStub* navigator, clients::FollowStub* follow,
              clients::DockStub* dock, clients::ExplorationStub* exploration);

    /**
     * @brief Handle a voice command and stream dispatch status.
     * @param[in] request Voice command request.
     * @param[in] stream_callback Stream sink.
     * @return false if rejected immediately.
     */
    bool HandleCommand(const proto::VoiceCommandRequest& request,
                       StreamCallback stream_callback);

    /** @brief Cancel the active voice-dispatched session. */
    void CancelActiveSession();

private:
    /**
     * @brief Build a voice command response frame.
     * @param[in] request Source request.
     * @param[in] status Voice status enum.
     * @param[in] success Ack success flag.
     * @param[in] final Terminal stream flag.
     * @param[in] message Optional ack message.
     */
    proto::VoiceCommandResponse MakeResponse(
        const proto::VoiceCommandRequest& request, proto::VoiceStatus status,
        bool success, bool final, const std::string& message = "") const;

    std::shared_ptr<TaskMuxer> muxer_;
    clients::NavigatorStub* navigator_{nullptr};
    clients::FollowStub* follow_{nullptr};
    clients::DockStub* dock_{nullptr};
    clients::ExplorationStub* exploration_{nullptr};

    mutable std::mutex mutex_;
    proto::VoiceCommandRequest last_request_;
    StreamCallback stream_callback_;
};

}  // namespace clients
}  // namespace grpc
}  // namespace bridge
}  // namespace autonomy
