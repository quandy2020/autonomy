/* Copyright 2026 The Navigation Authors (navigation) */
#pragma once

#include <cstdint>
#include <optional>

namespace navigation::safety {

/** @brief Track depth progress on a monotonic clock, independent of sim epoch.
 */
class DepthFreshness
{
public:
    /** @brief Repeated cached stamps do not renew the last received time. */
    void Observe(std::int64_t depth_stamp_ns, std::int64_t steady_now_ns) {
        if (depth_stamp_ns < 0 || steady_now_ns <= 0) {
            return;
        }
        if (!stamp_.has_value() || depth_stamp_ns != *stamp_) {
            stamp_ = depth_stamp_ns;
            received_ns_ = steady_now_ns;
        }
    }

    [[nodiscard]] bool Fresh(std::int64_t steady_now_ns) const {
        return stamp_.has_value() && steady_now_ns >= received_ns_ &&
               steady_now_ns - received_ns_ <= 200000000;
    }

    [[nodiscard]] std::int64_t ReceivedNs() const {
        return received_ns_;
    }

    void Reset() {
        stamp_.reset();
        received_ns_ = 0;
    }

private:
    std::optional<std::int64_t> stamp_;
    std::int64_t received_ns_{0};
};

}  // namespace navigation::safety
