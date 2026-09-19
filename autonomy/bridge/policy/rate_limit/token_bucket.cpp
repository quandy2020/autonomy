/*
 * Copyright 2026 The Openbot Authors
 */

#include "autonomy/bridge/policy/rate_limit/token_bucket.hpp"

#include <algorithm>
#include <cmath>

namespace autonomy {
namespace bridge {
namespace policy {

TokenBucket::TokenBucket(double qps, unsigned burst)
    : qps_(qps),
      capacity_(burst > 0
                    ? static_cast<double>(burst)
                    : std::max(1.0, std::ceil(std::max(0.0, qps)))),
      tokens_(capacity_),
      last_refill_(std::chrono::steady_clock::now()) {}

void TokenBucket::RefillLocked(std::chrono::steady_clock::time_point now) {
    if (qps_ <= 0.0) {
        return;
    }
    const double elapsed =
        std::chrono::duration<double>(now - last_refill_).count();
    if (elapsed <= 0.0) {
        return;
    }
    tokens_ = std::min(capacity_, tokens_ + elapsed * qps_);
    last_refill_ = now;
}

bool TokenBucket::TryAcquire() {
    if (qps_ <= 0.0) {
        return true;
    }
    std::lock_guard<std::mutex> lock(mutex_);
    RefillLocked(std::chrono::steady_clock::now());
    if (tokens_ < 1.0) {
        return false;
    }
    tokens_ -= 1.0;
    return true;
}

}  // namespace policy
}  // namespace bridge
}  // namespace autonomy
