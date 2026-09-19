/*
 * Copyright 2026 The Openbot Authors
 */

/**
 * @file idempotency.cpp
 * @brief Implementation of CommandIdempotencyCache TryBegin / Forget / Sweep.
 */

#include "autonomy/bridge/grpc/idempotency.hpp"

namespace autonomy {
namespace bridge {
namespace grpc {

CommandIdempotencyCache::CommandIdempotencyCache(const std::chrono::seconds ttl)
    : ttl_(ttl) {}

void CommandIdempotencyCache::Sweep() {
    const auto now = std::chrono::steady_clock::now();
    for (auto it = entries_.begin(); it != entries_.end();) {
        if (it->second.expires_at <= now) {
            it = entries_.erase(it);
        } else {
            ++it;
        }
    }
}

bool CommandIdempotencyCache::TryBegin(const std::string& cmd_id,
                                       const TaskType type) {
    if (cmd_id.empty()) {
        return true;
    }
    std::lock_guard<std::mutex> lock(mutex_);
    Sweep();
    const auto it = entries_.find(cmd_id);
    if (it != entries_.end()) {
        return false;
    }
    entries_[cmd_id] =
        Entry{type, std::chrono::steady_clock::now() + ttl_};
    return true;
}

void CommandIdempotencyCache::Forget(const std::string& cmd_id) {
    if (cmd_id.empty()) {
        return;
    }
    std::lock_guard<std::mutex> lock(mutex_);
    entries_.erase(cmd_id);
}

}  // namespace grpc
}  // namespace bridge
}  // namespace autonomy
