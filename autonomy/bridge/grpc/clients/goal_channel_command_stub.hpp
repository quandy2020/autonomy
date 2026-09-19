/*
 * Copyright 2026 The Openbot Authors
 */

/**
 * @file goal_channel_command_stub.hpp
 * @brief GoalChannelCommandStub + BRIDGE_CHANNEL_TRAITS macros.
 *
 * @details
 * Domain stubs inherit GoalChannelCommandStub<Traits> and expose thin
 * Handle* wrappers. Generated Traits structs include
 * AUTONOMY_SMART_PTR_DEFINITIONS(Name) via BRIDGE_CHANNEL_TRAITS_BEGIN and
 * declare Convert* / MakeResponse / IsTerminal (implemented in each domain
 * .cpp). Optional Traits::ShouldEmit / RejectReason are picked up by
 * GoalChannelStub.
 *
 * @par Ownership
 * Embeds GoalChannelStub by value (`channel_`). Domain UniquePtr owned by
 * DomainBundle.
 *
 * @par Threading
 * HandleRequest / Pause / Resume / Cancel are intended for the gRPC event
 * thread; must not block on Action accept waits.
 *
 * @par Invariants
 * - AcceptStart acquires muxer then ACK; feedback only while IsActive.
 * - Pause/Resume/Cancel write command goals via WriteCommand; no Action wait
 *   on the event thread.
 * - Traits supply kPause/kResume/kCancel and ConvertFeedback.
 * - Optional Traits::ShouldEmit / RejectReason are picked up by GoalChannelStub.
 *
 * @see GoalChannelStub
 * @see BRIDGE_CHANNEL_TRAITS
 */

#pragma once

#include <functional>
#include <memory>
#include <string>
#include "autolink/node/node.hpp"
#include "autonomy/bridge/grpc/clients/goal_channel_stub.hpp"
#include "autonomy/bridge/grpc/task_muxer.hpp"
#include "autonomy/bridge/grpc/task_types.hpp"
#include "autonomy/common/macros.hpp"

namespace autonomy {
namespace bridge {
namespace grpc {
namespace clients {

/**
 * @brief Fill goal.header task_id / task_type.
 *
 * @tparam Goal         Goal protobuf type with mutable_header().
 * @tparam TaskTypeEnum Task-type enum written into the header.
 * @param[in,out] goal         Goal message to mutate.
 * @param[in]     task_id      Task / goal id string.
 * @param[in]     task_type    Task type enum value.
 */
template <typename Goal, typename TaskTypeEnum>
void SetTaskHeader(Goal* goal, const std::string& task_id,
                   TaskTypeEnum task_type) {
    auto* header = goal->mutable_header();
    header->set_task_id(task_id);
    header->set_task_type(task_type);
}

/**
 * @brief GoalChannel domain stub: Dispatch + lifecycle WriteGoal.
 *
 * @tparam Traits GoalChannelStub traits, plus:
 * - `kPause` / `kResume` / `kCancel` (Goal::command)
 *
 * @par Ownership
 * Owns an embedded GoalChannelStub<Traits> (`channel_`). Pause /
 * Resume write command goals without clearing the session; Cancel also
 * ClearSession(true).
 *
 * @par Threading
 * Non-blocking on the gRPC event thread; feedback is relayed asynchronously.
 *
 * @note Domain stubs inherit this type and add thin Handle* facades.
 * @see GoalChannelStub
 * @see BRIDGE_CHANNEL_TRAITS
 */
template <typename Traits>
class GoalChannelCommandStub
{
public:
    /**
     * @brief Shared / weak / unique pointer aliases for GoalChannelCommandStub.
     */
    AUTONOMY_SMART_PTR_DEFINITIONS(GoalChannelCommandStub<Traits>)

    /**
     * @brief Rpc request type from Traits.
     */
    using Request = typename Traits::Request;

    /**
     * @brief Rpc / stream response type from Traits.
     */
    using Response = typename Traits::Response;

    /**
     * @brief Stream sink for ACK / feedback / terminal Response frames.
     */
    using StreamCallback = std::function<void(const Response&)>;

    /**
     * @brief Construct with shared node and optional muxer.
     *
     * @param[in] node  Autolink node for pub/sub.
     * @param[in] muxer Shared task muxer (may be null).
     */
    GoalChannelCommandStub(std::shared_ptr<autolink::Node> node,
                           TaskMuxer::SharedPtr muxer)
        : channel_(std::move(node), std::move(muxer)) {}

    /**
     * @brief Accept START-style command: acquire muxer, ACK, bind feedback.
     *
     * @param[in] request         Rpc command request.
     * @param[in] stream_callback Stream sink for ACK / feedback / terminal.
     * @return                    false if rejected before schedule (reject already emitted).
     */
    bool HandleRequest(const Request& request, StreamCallback stream_callback) {
        return channel_.Dispatch(
            request, std::move(stream_callback), /*acquire=*/true,
            [&](const StreamCallback& emit) {
                channel_.SetSessionActive(true);
                emit(Traits::MakeResponse(request, true, false, ""));
                return true;
            });
    }

    /**
     * @brief Publish the Traits pause command goal.
     *
     * @param[in] goal_id Unused; reserved for API symmetry.
     */
    void PauseGoal(const std::string& /*goal_id*/ = {}) {
        WriteCommand(Traits::kPause);
    }

    /**
     * @brief Publish the Traits resume command goal.
     *
     * @param[in] goal_id Unused; reserved for API symmetry.
     */
    void ResumeGoal(const std::string& /*goal_id*/ = {}) {
        WriteCommand(Traits::kResume);
    }

    /**
     * @brief Publish cancel command and clear the session / muxer slot.
     *
     * @param[in] goal_id Unused; reserved for API symmetry.
     */
    void CancelGoal(const std::string& /*goal_id*/ = {}) {
        WriteCommand(Traits::kCancel);
        channel_.ClearSession(true);
    }

    /**
     * @brief Alias for CancelGoal() (session teardown).
     */
    void CancelSession() { CancelGoal(); }

    /**
     * @brief Whether the embedded channel reports an active session.
     *
     * @return channel_.IsActive().
     */
    bool IsActive() const { return channel_.IsActive(); }

protected:
    /**
     * @brief Embedded GoalChannel session (goal writer + feedback reader).
     *
     * @details Domain Handle* wrappers call channel_.Dispatch / WriteGoal;
     * lifecycle helpers use WriteCommand on this member.
     */
    GoalChannelStub<Traits> channel_;

    /**
     * @brief Write a command-only goal (no acquire).
     *
     * @tparam Command Traits command enum / integral type.
     * @param[in] command Command value assigned to goal.command.
     */
    template <typename Command>
    void WriteCommand(Command command) {
        typename Traits::Goal goal;
        goal.set_command(command);
        channel_.WriteGoal(goal);
    }
};

/**
 * @brief Open a GoalChannel traits struct (type aliases + Convert* decls).
 *
 * Callers may append optional members (kPause, ShouldEmit, …) then
 * BRIDGE_CHANNEL_TRAITS_END().
 *
 * @note Injects AUTONOMY_SMART_PTR_DEFINITIONS(Name) into every generated
 * traits struct body.
 */
#define BRIDGE_CHANNEL_TRAITS_BEGIN(Name, GoalT, FeedbackT, RequestT,     \
                                    ResponseT, TaskTypeV, GoalCh,         \
                                    FeedbackCh)                           \
    struct Name {                                                         \
        AUTONOMY_SMART_PTR_DEFINITIONS(Name)                              \
        /** @brief Goal protobuf published on the goal topic. */          \
        using Goal = GoalT;                                               \
        /** @brief Feedback protobuf received on the feedback topic. */   \
        using Feedback = FeedbackT;                                       \
        /** @brief Rpc request type for ConvertToGoal. */                 \
        using Request = RequestT;                                         \
        /** @brief Rpc / stream response type. */                         \
        using Response = ResponseT;                                       \
        static constexpr TaskType kTaskType = TaskTypeV;                  \
        static constexpr const char* kGoalTopic = GoalCh;                 \
        static constexpr const char* kFeedbackTopic = FeedbackCh;         \
        static Goal ConvertToGoal(const Request& request);                \
        static Response ConvertFromFeedback(const Feedback& feedback,     \
                                            const Request& last);         \
        static Response MakeResponse(const Request& request, bool success,\
                                     bool final, const std::string& message); \
        static bool IsTerminal(const Feedback& feedback)

#define BRIDGE_CHANNEL_TRAITS_END() \
    }

/**
 * @brief Traits without lifecycle command constants (append kPause… yourself).
 */
#define BRIDGE_CHANNEL_TRAITS_BASE(Name, GoalT, FeedbackT, RequestT,      \
                                   ResponseT, TaskTypeV, GoalCh,          \
                                   FeedbackCh)                            \
    BRIDGE_CHANNEL_TRAITS_BEGIN(Name, GoalT, FeedbackT, RequestT,         \
                                ResponseT, TaskTypeV, GoalCh, FeedbackCh) \
    ;                                                                     \
    BRIDGE_CHANNEL_TRAITS_END()

/**
 * @brief Declare GoalChannel traits + Pause/Resume/Cancel command constants.
 *
 * Domain .cpp still implements ConvertToGoal / ConvertFromFeedback /
 * MakeResponse / IsTerminal.
 */
#define BRIDGE_CHANNEL_TRAITS(Name, GoalT, FeedbackT, RequestT, ResponseT, \
                              TaskTypeV, GoalCh, FeedbackCh, PauseCmd,     \
                              ResumeCmd, CancelCmd)                        \
    BRIDGE_CHANNEL_TRAITS_BEGIN(Name, GoalT, FeedbackT, RequestT,          \
                                ResponseT, TaskTypeV, GoalCh, FeedbackCh); \
        static constexpr auto kPause = PauseCmd;                           \
        static constexpr auto kResume = ResumeCmd;                         \
        static constexpr auto kCancel = CancelCmd;                         \
    BRIDGE_CHANNEL_TRAITS_END()

}  // namespace clients
}  // namespace grpc
}  // namespace bridge
}  // namespace autonomy
