/*
 * Copyright 2026 The Openbot Authors
 */

/**
 * @file rpc_voice_handlers.hpp
 * @brief VoiceService RpcHandlers: Execute command stream.
 *
 * @details
 * Single streaming RPC. Forwards to Context::voice() (VoiceStub). Stub
 * GoalChannel topics are @c kVoiceGoal / @c kVoiceFeedback
 * (`/autonomy/task/voice/{goal,feedback}`). There is no lifecycle /
 * GetStatus surface in this header — cancel is inherited only if registered
 * elsewhere via CancelRegistry / Estop.
 *
 * Generated types (SMART_PTR via BRIDGE_STREAM typedef path):
 * - RpcVoiceExecuteHandler — HandleCommand / IsVoiceTerminal
 *
 * Invariants:
 * - Execute finishes on IsVoiceTerminal (failed / succeeded / cancelled).
 * - Handlers never own VoiceTask state; VoiceStub / TaskServer do.
 * - Ownership: per-RPC handler; VoiceStub owned by Context.
 * - Threading: gRPC completion queue; feedback relayed asynchronously.
 *
 * @note VoiceTraits maps Pause/Resume/Cancel all to VOICE_CMD_CANCEL.
 * @see VoiceStub
 * @see handler_templates.hpp
 */

#pragma once

#include "autonomy/bridge/grpc/handlers/handler_templates.hpp"
#include <automsgs/rpcs/voice.pb.h>

namespace autonomy {
namespace bridge {
namespace grpc {
namespace handlers {

/**
 * @brief VoiceService/Execute — stream VoiceCommandResponse until terminal.
 */
BRIDGE_STREAM(
    RpcVoiceExecuteHandler, ::automsgs::rpcs::voice::VoiceCommandRequest,
    autonomy::common::async_grpc::Stream<
        ::automsgs::rpcs::voice::VoiceCommandResponse>,
    "/automsgs.rpcs.voice.VoiceService/Execute", &Context::voice,
    &clients::VoiceStub::HandleCommand, &IsVoiceTerminal);

}  // namespace handlers
}  // namespace grpc
}  // namespace bridge
}  // namespace autonomy
