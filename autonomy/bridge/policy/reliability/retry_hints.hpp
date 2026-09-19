/*
 * Copyright 2026 The Openbot Authors
 */

/**
 * @file retry_hints.hpp
 * @brief Client-side retry guidance (documentation in code form).
 */

#pragma once

namespace autonomy {
namespace bridge {
namespace policy {

/**
 * @brief Whether a method is generally safe to retry at the client.
 *
 * START-like commands should rely on goal_id idempotency instead of blind
 * retries. This helper is advisory only (not enforced server-side).
 */
inline bool IsIdempotentReadMethod(const char* /*method_full_name*/) {
    // First phase: treat all as non-auto-retry at the server; clients decide.
    return false;
}

}  // namespace policy
}  // namespace bridge
}  // namespace autonomy
