/*
 * Copyright 2026 The Openbot Authors
 */

/**
 * @file charge_stub.hpp
 * @brief ChargeStub: GoalChannel adapter for auto-dock / undock (ChargingTask).
 *
 * @details
 * Maps ChargeService RPCs onto `/autonomy/task/charging/{goal,feedback}`
 * (see @c kChargingGoal / @c kChargingFeedback in bridge/constants.hpp;
 * legacy aliases @c kChargingGoalChannel / @c kChargingFeedbackChannel).
 * Wire messages are `automsgs/task/charging.pb.h` (`ChargingGoal` /
 * `ChargingFeedback`); Bridge does **not** include `autonomy/task` headers.
 *
 * @par Lifecycle
 * - Return (dock) → HandleReturn → GoalChannelCommandStub::HandleRequest
 * - Leave (undock) → HandleLeave (builds UNDOCK goal in .cpp)
 * - Pause / Resume / Cancel → DOCK_CMD_* via base WriteCommand
 *
 * @par Invariants
 * - Muxer slot type is TASK_TYPE_DOCK.
 * - Convert* / MakeResponse / IsTerminal live in charge_stub.cpp.
 * - Leave uses the same ChargeResponse stream shape as Return.
 *
 * @par Ownership
 * UniquePtr owned by DomainBundle; CancelRegistry captures non-owning Stub*.
 *
 * @par Threading
 * intended for the gRPC event thread; must not block on Action
 * results. Feedback is relayed asynchronously while IsActive().
 *
 * @see GoalChannelCommandStub
 * @see rpc_charge_handlers.hpp
 */

#pragma once

#include "autonomy/bridge/grpc/clients/goal_channel_command_stub.hpp"
#include "autonomy/bridge/constants.hpp"
#include <automsgs/rpcs/charge.pb.h>
#include <automsgs/task/charging.pb.h>

namespace autonomy {
namespace bridge {
namespace grpc {
namespace clients {

/**
 * @brief GoalChannel traits for ChargingTask (generated + DOCK_CMD_*).
 *
 * @details
 * Declares ConvertToGoal / ConvertFromFeedback / MakeResponse /
 * IsTerminal (implemented in charge_stub.cpp) and binds Pause/Resume/Cancel
 * to DOCK_CMD_PAUSE / RESUME / CANCEL. Topics:
 * - Goal: @c kChargingGoal (`/autonomy/task/charging/goal`)
 * - Feedback: @c kChargingFeedback (`/autonomy/task/charging/feedback`)
 *
 * @note Request type for the streaming Convert path is ReturnRequest;
 *       Leave builds its goal separately in HandleLeave.
 * @warning Muxer type TASK_TYPE_DOCK must stay aligned with TaskServer.
 */
BRIDGE_CHANNEL_TRAITS(
    ChargeTraits,
    ::autonomy::task::proto::ChargingGoal,
    ::autonomy::task::proto::ChargingFeedback,
    ::automsgs::rpcs::charge::ReturnRequest,
    ::automsgs::rpcs::charge::ChargeResponse,
    ::autonomy::bridge::grpc::TASK_TYPE_DOCK,
    ::autonomy::bridge::kChargingGoal,
    ::autonomy::bridge::kChargingFeedback,
    ::autonomy::task::proto::DOCK_CMD_PAUSE,
    ::autonomy::task::proto::DOCK_CMD_RESUME,
    ::autonomy::task::proto::DOCK_CMD_CANCEL);

/**
 * @brief ChargeService Return / Leave via GoalChannel (Leave = UNDOCK).
 *
 * @details
 * Thin facade over @ref GoalChannelCommandStub<ChargeTraits>.
 * Handlers call HandleReturn / HandleLeave for streams and inherit
 * PauseGoal / ResumeGoal / CancelGoal from the base.
 *
 * @par Threading
 * intended for the gRPC event thread; must not block on Action
 * results. Feedback is relayed asynchronously while IsActive().
 *
 * @par Ownership
 * UniquePtr owned by DomainBundle; CancelRegistry captures non-owning Stub*.
 *
 * @note Convert* / MakeResponse / IsTerminal live in charge_stub.cpp.
 * @warning HandleReturn / HandleLeave return false if gated out before write.
 * @see rpc_charge_handlers.hpp
 */
class ChargeStub : public GoalChannelCommandStub<ChargeTraits>
{
public:
    /**
     * @brief Shared / weak / unique pointer aliases.
     */
    AUTONOMY_SMART_PTR_DEFINITIONS(ChargeStub)

    /**
     * @brief Inherit GoalChannelCommandStub constructors (node + muxer).
     */
    using GoalChannelCommandStub::GoalChannelCommandStub;

    /**
     * @brief Start auto-return-to-dock and stream ChargeResponse frames.
     *
     * @param[in] request         ReturnRequest (goal_id, dock pose / policy fields).
     * @param[in] stream_callback Sink for ACK / feedback / terminal frames.
     * @return                    false if gated out before write (reject already emitted).
     */
    bool HandleReturn(const ::automsgs::rpcs::charge::ReturnRequest& request,
                      StreamCallback stream_callback) {
        return HandleRequest(request, std::move(stream_callback));
    }

    /**
     * @brief Start undock / leave-charger and stream ChargeResponse frames.
     *
     * @param[in] request         LeaveRequest (goal_id, leave policy).
     * @param[in] stream_callback Sink for ACK / feedback / terminal frames.
     * @return                    false if gated out before write (reject already emitted).
     *
     * @note Implemented in charge_stub.cpp: builds ChargingGoal with UNDOCK
     *       command then Dispatches like HandleRequest.
     */
    bool HandleLeave(const ::automsgs::rpcs::charge::LeaveRequest& request,
                     StreamCallback stream_callback);
};

}  // namespace clients
}  // namespace grpc
}  // namespace bridge
}  // namespace autonomy
