/*
 * Copyright 2026 The Openbot Authors
 */

/**
 * @file idempotency.hpp
 * @brief Short-lived goal_id / cmd_id dedupe for START-like Rpc commands.
 *
 * Session Gate (BackgroundCommandSession) calls TryBegin before muxer
 * acquire; Forget drops ids that were rejected after TryBegin or finished
 * Execute without keeping the slot. GoalChannel domains may skip this cache
 * when their Action layer already dedupes goals.
 *
 * @par Invariants
 * - TryBegin rejects duplicates within TTL; Forget drops early rejects.
 * - Used by Session Gate only; GoalChannel domains may skip if unused.
 * - Empty cmd_id always allowed (no cache entry written).
 * - Sweep runs lazily inside TryBegin and may be called explicitly in tests.
 */

#pragma once

#include <chrono>
#include <mutex>
#include <string>
#include <unordered_map>

#include "autonomy/bridge/grpc/task_types.hpp"
#include "autonomy/common/macros.hpp"

namespace autonomy {
namespace bridge {
namespace grpc {

/**
 * @brief Remembers recent cmd_id values to reject duplicate START commands.
 *
 * Bounded by time-to-live only (no hard max size); Sweep / lazy expiry remove
 * stale entries. Associates each remembered id with the TaskType that began
 * it for debugging / future policy hooks.
 *
 * @par Threading
 * all public methods take mutex_; safe from handler + worker
 * threads that share one cache instance via Context.
 *
 * @par Ownership
 * Context owns UniquePtr; Session / Teleop hold a non-owning raw
 * pointer injected at construction.
 */
class CommandIdempotencyCache
{
public:
    /**
     * @brief Shared / weak / unique pointer aliases.
     */
    AUTONOMY_SMART_PTR_DEFINITIONS(CommandIdempotencyCache)

    /**
     * @brief Construct a cmd_id cache with a time-to-live.
     *
     * @param[in] ttl How long a remembered cmd_id blocks duplicates
     *               (default 60s).
     */
    explicit CommandIdempotencyCache(
        std::chrono::seconds ttl = std::chrono::seconds(60));

    /**
     * @brief Check whether @p cmd_id may start a new task.
     *
     * Empty @p cmd_id always returns true without recording. Otherwise runs
     * a lazy Sweep, then inserts @p cmd_id with expiry now+ttl when absent.
     *
     * @param[in] cmd_id Command id to record (empty always allowed).
     * @param[in] type   Task type associated with this start.
     * @return           true if @p cmd_id is empty or not a recent duplicate.
     *
     * @note On false, the caller should reject the RPC without acquiring the
     * muxer; do not call Forget for a failed TryBegin (nothing inserted).
     */
    bool TryBegin(const std::string& cmd_id, TaskType type);

    /**
     * @brief Drop @p cmd_id (e.g. after reject before accept, or Execute end).
     *
     * @param[in] cmd_id Command id to forget; no-op if absent or empty.
     *
     * @note BackgroundCommandSession Forget()s when muxer acquire fails after
     * TryBegin, and when Execute returns keep_slot=false.
     */
    void Forget(const std::string& cmd_id);

    /**
     * @brief Expire stale entries (also runs lazily inside TryBegin).
     *
     * Erases every Entry whose expires_at is not after steady_clock::now().
     */
    void Sweep();

private:
    /**
     * @brief Cached cmd_id entry with expiry.
     *
     * POD snapshot stored in the unordered_map value. type records which
     * TaskType first claimed the id; expires_at uses steady_clock so TTL is
     * immune to wall-clock jumps.
     */
    struct Entry {
        AUTONOMY_SMART_PTR_DEFINITIONS(Entry)

        /**
         * @brief TaskType that first claimed this cmd_id via TryBegin.
         */
        TaskType type{TASK_TYPE_NONE};

        /**
         * @brief Steady-clock expiry; Sweep drops entries at or before now.
         */
        std::chrono::steady_clock::time_point expires_at;
    };

    /**
     * @brief Time-to-live applied as now+ttl when inserting a new cmd_id.
     */
    const std::chrono::seconds ttl_;

    /**
     * @brief Guards entries_ for concurrent TryBegin / Forget / Sweep.
     */
    mutable std::mutex mutex_;

    /**
     * @brief cmd_id → Entry map of recently begun START-like commands.
     */
    std::unordered_map<std::string, Entry> entries_;
};

}  // namespace grpc
}  // namespace bridge
}  // namespace autonomy
