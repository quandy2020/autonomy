/*
 * Copyright 2026 The Openbot Authors
 *
 * Short-lived cmd_id dedupe for START-like bridge commands.
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
 * @brief Remembers recent `cmd_id` values to reject duplicate START commands.
 *
 * Lock order: never hold a Stub mutex while calling into this cache if the
 * caller already holds `TaskMuxer` — acquire muxer first, then idempotency.
 */
class CommandIdempotencyCache
{
public:
    AUTONOMY_SMART_PTR_DEFINITIONS(CommandIdempotencyCache)

    explicit CommandIdempotencyCache(
        std::chrono::seconds ttl = std::chrono::seconds(60));

    /**
     * @brief Check whether @p cmd_id may start a new task.
     * @return true if @p cmd_id is empty or not a recent duplicate.
     */
    bool TryBegin(const std::string& cmd_id, TaskType type);

    /** @brief Drop @p cmd_id (e.g. after terminal failure before accept). */
    void Forget(const std::string& cmd_id);

    /** @brief Expire stale entries (also runs lazily inside TryBegin). */
    void Sweep();

private:
    struct Entry {
        TaskType type{TASK_TYPE_NONE};
        std::chrono::steady_clock::time_point expires_at;
    };

    const std::chrono::seconds ttl_;
    mutable std::mutex mutex_;
    std::unordered_map<std::string, Entry> entries_;
};

}  // namespace grpc
}  // namespace bridge
}  // namespace autonomy
