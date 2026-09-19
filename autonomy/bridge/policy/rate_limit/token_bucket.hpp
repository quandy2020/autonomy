/*
 * Copyright 2026 The Openbot Authors
 */

/**
 * @file token_bucket.hpp
 * @brief Process-wide token-bucket rate limiter.
 */

#pragma once

#include <chrono>
#include <mutex>

#include "autonomy/common/macros.hpp"

namespace autonomy {
namespace bridge {
namespace policy {

/**
 * @brief Simple token bucket (docs/02_policy.md).
 */
class TokenBucket
{
public:
    AUTONOMY_SMART_PTR_DEFINITIONS(TokenBucket)

    /**
     * @brief Construct a bucket.
     *
     * @param[in] qps   Refill rate (tokens per second). <=0 disables limiting.
     * @param[in] burst Max tokens; 0 → max(1, ceil(qps)).
     */
    TokenBucket(double qps, unsigned burst);

    /**
     * @brief Try to consume one token.
     *
     * @return true if allowed; false if exhausted.
     */
    bool TryAcquire();

    /** @brief Whether limiting is active. */
    bool enabled() const { return qps_ > 0.0; }

private:
    void RefillLocked(std::chrono::steady_clock::time_point now);

    const double qps_;
    const double capacity_;
    double tokens_{0.0};
    std::chrono::steady_clock::time_point last_refill_;
    mutable std::mutex mutex_;
};

}  // namespace policy
}  // namespace bridge
}  // namespace autonomy
