/*
 * Copyright 2026 The Openbot Authors
 */

/**
 * @file session.hpp
 * @brief Blocking-command session: gate → ACK → worker Execute → optional Release.
 *
 * Used by Action-backed domain stubs that must not block the gRPC event
 * thread. Start() runs the Session Gate (estop / idempotency / muxer), emits
 * Accept ACK on the calling thread, then schedules Execute on WorkScheduler.
 *
 * @par Invariants
 * - Never block the gRPC event thread; Execute runs on WorkScheduler.
 * - Accept ACK is emitted before Execute is scheduled.
 * - Return false from Start ⇒ caller must not assume muxer ownership.
 * - Execute returning false (or throwing) Releases the muxer slot and Forget()s
 * cmd_id when release_muxer_when_done was set by Start.
 */

#pragma once

#include <functional>
#include <memory>
#include <string>
#include <utility>

#include "autonomy/bridge/grpc/clients/command_dispatch.hpp"
#include "autonomy/bridge/grpc/idempotency.hpp"
#include "autonomy/bridge/grpc/task_muxer.hpp"
#include "autonomy/bridge/grpc/task_types.hpp"
#include "autonomy/bridge/grpc/work_scheduler.hpp"
#include "autonomy/common/macros.hpp"
#include "autolink/common/log.hpp"

namespace autonomy {
namespace bridge {
namespace grpc {

/**
 * @brief Background command session for Action-backed stubs.
 *
 * Domain stubs supply: validation, Accept ACK, Execute body. This type owns
 * no long-lived state beyond the injected scheduler / muxer / idempotency
 * pointers; each Start / ScheduleOnly is independent.
 *
 * @tparam Response Stream frame type (automsgs.rpcs response).
 *
 * @par Threading
 * Start / ScheduleOnly run on the gRPC handler thread; Execute
 * runs on a WorkScheduler worker. StreamCallback may be invoked from either.
 *
 * @par Ownership
 * non-owning WorkScheduler* and CommandIdempotencyCache*; shared
 * ownership of TaskMuxer via shared_ptr (kept alive across the scheduled
 * lambda).
 */
template <typename Response>
class BackgroundCommandSession
{
public:
    /**
     * @brief Shared / weak / unique pointer aliases.
     */
    AUTONOMY_SMART_PTR_DEFINITIONS(BackgroundCommandSession<Response>)

    /**
     * @brief Stream sink for ACK / progress / terminal response frames.
     */
    using StreamCallback = std::function<void(const Response&)>;

    /**
     * @brief Execute body scheduled on the work pool.
     *
     * @param emit Stream sink (same callback passed to Start).
     * @return     true to keep muxer slot after Execute; false to Release and
     *            Forget cmd_id when Start acquired the slot.
     */
    using ExecuteFn = std::function<bool(const StreamCallback& emit)>;

    /**
     * @brief Build a reject / error response frame from a human message.
     *
     * @param message Gate-failure reason (estop, duplicate cmd_id, busy, …).
     * @return        Response frame written via StreamCallback.
     */
    using RejectBuilder = std::function<Response(const std::string&)>;

    /**
     * @brief Construct a session bound to scheduler, muxer, and task type.
     *
     * @param[in] scheduler Background work pool (non-owning; must outlive
     *                       every scheduled Execute).
     * @param[in] muxer       Shared exclusive-task muxer (may be null in tests).
     * @param[in] task_type   Muxer slot type for this command family.
     * @param[in] idempotency Optional cmd_id dedupe cache (non-owning).
     */
    BackgroundCommandSession(WorkScheduler* scheduler,
                             TaskMuxer::SharedPtr muxer,
                             TaskType task_type,
                             CommandIdempotencyCache* idempotency = nullptr)
        : scheduler_(scheduler),
          muxer_(std::move(muxer)),
          task_type_(task_type),
          idempotency_(idempotency) {}

    /**
     * @brief Gate → Accept ACK → schedule Execute (acquires muxer).
     *
     * Order: null-emit check → scheduler check → RejectOnEstop →
     * TryBegin(cmd_id) → TryAcquire → emit accept_ack → ScheduleExecute with
     * release_muxer_when_done=true.
     *
     * @param[in] emit           Stream sink for ACK / progress / terminal frames.
     * @param[in] accept_ack     Immediate accept frame (moved after gate passes).
     * @param[in] reject_builder Builds reject frames on gate failure.
     * @param[in] execute        Blocking work run on the pool.
     * @param[in] cmd_id         Command id for muxer / idempotency.
     * @param[in] client_id      Optional client id recorded in the muxer snapshot.
     * @return                   false if gated out before scheduling (caller must not assume
     *                          muxer ownership).
     *
     * @note On muxer acquire failure after a successful TryBegin, Forget()s
     * @p cmd_id so the client may retry with the same id.
     */
    bool Start(StreamCallback emit, Response accept_ack,
               RejectBuilder reject_builder, ExecuteFn execute,
               const std::string& cmd_id,
               const std::string& client_id = {}) {
        if (!emit) {
            AERROR << "BackgroundCommandSession: null stream callback";
            return false;
        }
        if (!scheduler_) {
            emit(reject_builder("work scheduler unavailable"));
            return false;
        }
        auto fail = [&](const std::string& message) {
            emit(reject_builder(message));
        };
        if (clients::RejectOnEstop(muxer_, fail)) {
            return false;
        }
        if (idempotency_ && !idempotency_->TryBegin(cmd_id, task_type_)) {
            fail("duplicate cmd_id");
            return false;
        }
        if (muxer_ && !muxer_->TryAcquire(task_type_, cmd_id, client_id)) {
            if (idempotency_) {
                idempotency_->Forget(cmd_id);
            }
            fail("another task is active");
            return false;
        }
        emit(std::move(accept_ack));
        ScheduleExecute(std::move(emit), std::move(execute),
                        /*release_muxer_when_done=*/true, cmd_id);
        return true;
    }

    /**
     * @brief Like @ref Start, extracting ids from an Rpc request with `goal_id()`.
     *
     * @tparam Request        Request type exposing `goal_id()`.
     * @param[in] request        Incoming request carrying goal_id.
     * @param[in] emit           Stream sink for ACK / progress / terminal frames.
     * @param[in] accept_ack     Immediate accept frame.
     * @param[in] reject_builder Builds reject frames on gate failure.
     * @param[in] execute        Blocking work run on the pool.
     * @return                   false if gated out before scheduling.
     *
     * @note client_id is passed as empty; use the six-arg Start overload when
     * the muxer snapshot must record a client.
     */
    template <typename Request>
    bool Start(const Request& request, StreamCallback emit, Response accept_ack,
               RejectBuilder reject_builder, ExecuteFn execute) {
        return Start(std::move(emit), std::move(accept_ack),
                     std::move(reject_builder), std::move(execute),
                     request.goal_id(), {});
    }

    /**
     * @brief Immediate progress ACK + schedule (no muxer acquire/release).
     *
     * For follow-up work that must not claim the exclusive slot (e.g. status
     * polling helpers). Does not touch idempotency.
     *
     * @param[in] emit         Stream sink.
     * @param[in] progress_ack Immediate progress frame.
     * @param[in] execute      Blocking work run on the pool.
     * @return                 false if emit or scheduler is missing.
     */
    bool ScheduleOnly(StreamCallback emit, Response progress_ack,
                      ExecuteFn execute) {
        if (!emit || !scheduler_) {
            return false;
        }
        emit(std::move(progress_ack));
        ScheduleExecute(std::move(emit), std::move(execute),
                        /*release_muxer_when_done=*/false, {});
        return true;
    }

private:
    /**
     * @brief Enqueue Execute and optionally release muxer / idempotency.
     *
     * Catches std::exception and unknown exceptions from Execute, logging and
     * treating them as keep_slot=false so the slot is released when requested.
     *
     * @param[in] emit                    Stream sink.
     * @param[in] execute                 Blocking work body.
     * @param[in] release_muxer_when_done Whether to Release after Execute when
     *                                   keep_slot is false.
     * @param[in] cmd_id Command id for idempotency Forget.
     */
    void ScheduleExecute(StreamCallback emit, ExecuteFn execute,
                         bool release_muxer_when_done,
                         const std::string& cmd_id) {
        auto muxer = muxer_;
        auto* idempotency = idempotency_;
        const auto task_type = task_type_;
        scheduler_->Schedule(
            [execute = std::move(execute), emit = std::move(emit), muxer,
             idempotency, task_type, release_muxer_when_done,
             cmd_id]() mutable {
                bool keep_slot = false;
                try {
                    keep_slot = execute(emit);
                } catch (const std::exception& ex) {
                    AERROR << "BackgroundCommandSession execute threw: "
                           << ex.what();
                    keep_slot = false;
                } catch (...) {
                    AERROR << "BackgroundCommandSession execute threw unknown";
                    keep_slot = false;
                }
                if (release_muxer_when_done && !keep_slot) {
                    if (muxer) {
                        muxer->Release(task_type);
                    }
                    if (idempotency) {
                        idempotency->Forget(cmd_id);
                    }
                }
            });
    }

    /**
     * @brief Non-owning work pool used to Schedule Execute bodies.
     */
    WorkScheduler* scheduler_{nullptr};

    /**
     * @brief Shared exclusive-task muxer acquired by Start (may be null).
     */
    TaskMuxer::SharedPtr muxer_{nullptr};

    /**
     * @brief Muxer TaskType claimed by this session family.
     */
    TaskType task_type_{TASK_TYPE_NONE};

    /**
     * @brief Optional non-owning cmd_id dedupe cache (Session Gate TryBegin).
     */
    CommandIdempotencyCache* idempotency_{nullptr};
};

}  // namespace grpc
}  // namespace bridge
}  // namespace autonomy
