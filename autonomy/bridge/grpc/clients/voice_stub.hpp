/*
 * Copyright 2026 The Openbot Authors
 */

/**
 * @file voice_stub.hpp
 * @brief VoiceStub: GoalChannel adapter for VoiceService → VoiceTask.
 *
 * @details
 * Maps VoiceService Execute onto `/autonomy/task/voice/{goal,feedback}`
 * (see @c kVoiceGoal / @c kVoiceFeedback in bridge/constants.hpp).
 * Wire messages are `automsgs/task/voice.pb.h` (`VoiceGoal` / `VoiceFeedback`);
 * Bridge does **not** include `autonomy/task` headers.
 *
 * @par Lifecycle
 * - Execute → HandleCommand → GoalChannelCommandStub::HandleRequest
 * - Pause / Resume / Cancel all map to VOICE_CMD_CANCEL (voice has no distinct
 * pause/resume wire commands; traits bind all three to cancel)
 *
 * @par Invariants
 * - Muxer slot type is TASK_TYPE_VOICE.
 * - Convert* / MakeResponse / IsTerminal live in voice_stub.cpp.
 * - PauseGoal / ResumeGoal inherited from base still publish VOICE_CMD_CANCEL.
 *
 * @par Ownership
 * UniquePtr owned by DomainBundle; CancelRegistry captures non-owning Stub*.
 *
 * @par Threading
 * gRPC event thread for HandleCommand; feedback relayed async.
 *
 * @see GoalChannelCommandStub
 * @see rpc_voice_handlers.hpp
 */

#pragma once

#include "autonomy/bridge/grpc/clients/goal_channel_command_stub.hpp"
#include "autonomy/bridge/constants.hpp"
#include <automsgs/rpcs/voice.pb.h>
#include <automsgs/task/voice.pb.h>

namespace autonomy {
namespace bridge {
namespace grpc {
namespace clients {

/**
 * @brief GoalChannel traits for VoiceTask (generated + VOICE_CMD_CANCEL).
 *
 * @details
 * Declares ConvertToGoal / ConvertFromFeedback / MakeResponse /
 * IsTerminal (implemented in voice_stub.cpp). Pause/Resume/Cancel constants
 * are all VOICE_CMD_CANCEL because VoiceTask does not expose separate
 * pause/resume commands on the wire.
 *
 * Topics:
 * - Goal: @c kVoiceGoal (`/autonomy/task/voice/goal`)
 * - Feedback: @c kVoiceFeedback (`/autonomy/task/voice/feedback`)
 *
 * @note Request is VoiceCommandRequest; Response is VoiceCommandResponse.
 * @warning Calling PauseGoal / ResumeGoal still cancels the voice session.
 */
BRIDGE_CHANNEL_TRAITS(
    VoiceTraits,
    ::autonomy::task::proto::VoiceGoal,
    ::autonomy::task::proto::VoiceFeedback,
    ::automsgs::rpcs::voice::VoiceCommandRequest,
    ::automsgs::rpcs::voice::VoiceCommandResponse,
    ::autonomy::bridge::grpc::TASK_TYPE_VOICE,
    ::autonomy::bridge::kVoiceGoal,
    ::autonomy::bridge::kVoiceFeedback,
    ::autonomy::task::proto::VOICE_CMD_CANCEL,
    ::autonomy::task::proto::VOICE_CMD_CANCEL,
    ::autonomy::task::proto::VOICE_CMD_CANCEL);

/**
 * @brief VoiceService Execute via GoalChannel (VoiceTask).
 *
 * @details
 * Thin facade over @ref GoalChannelCommandStub<VoiceTraits>.
 * Handlers call HandleCommand for the Execute stream and inherit
 * PauseGoal / ResumeGoal / CancelGoal from the base (all cancel on the wire).
 *
 * @par Threading
 * intended for the gRPC event thread; must not block on Action
 * results. Feedback is relayed asynchronously while IsActive().
 *
 * @par Ownership
 * UniquePtr owned by DomainBundle; CancelRegistry captures non-owning Stub*.
 *
 * @note Convert* / MakeResponse / IsTerminal live in voice_stub.cpp.
 * @warning HandleCommand returns false if gated out before write.
 * @see rpc_voice_handlers.hpp
 */
class VoiceStub : public GoalChannelCommandStub<VoiceTraits>
{
public:
    /**
     * @brief Shared / weak / unique pointer aliases.
     */
    AUTONOMY_SMART_PTR_DEFINITIONS(VoiceStub)

    /**
     * @brief Inherit GoalChannelCommandStub constructors (node + muxer).
     */
    using GoalChannelCommandStub::GoalChannelCommandStub;

    /**
     * @brief Accept a voice command and stream VoiceTask feedback.
     *
     * Converts @p request to VoiceGoal, claims the muxer slot, writes the goal
     * on @c kVoiceGoal, and relays feedback until IsTerminal.
     *
     * @param[in] request         VoiceCommandRequest (utterance / intent / goal_id).
     * @param[in] stream_callback Sink for ACK / feedback / terminal frames.
     * @return                    false if rejected before write (reject already emitted).
     */
    bool HandleCommand(
        const ::automsgs::rpcs::voice::VoiceCommandRequest& request,
        StreamCallback stream_callback) {
        return HandleRequest(request, std::move(stream_callback));
    }
};

}  // namespace clients
}  // namespace grpc
}  // namespace bridge
}  // namespace autonomy
