/*
 * Copyright 2025 The Openbot Authors (duyongquan)
 */

#pragma once

#include <chrono>
#include <memory>
#include <thread>

#include "behaviortree_cpp/behavior_tree.h"

namespace autonomy {
namespace tasks {
namespace behavior_tree {

/** Steady-clock rate limiter for BT main loop (ported from nav2 LoopRate). */
class LoopRate
{
public:
    explicit LoopRate(std::chrono::milliseconds period, BT::Tree* tree)
        : period_(period), last_interval_(Clock::now()), tree_(tree) {}

    bool sleep() {
        const auto now = Clock::now();
        auto next_interval = last_interval_ + period_;
        if (now < last_interval_) {
            next_interval = now + period_;
        }
        last_interval_ += period_;
        if (next_interval <= now) {
            if (now > next_interval + period_) {
                last_interval_ = now + period_;
            }
            return false;
        }
        const auto time_to_sleep = next_interval - now;
        if (tree_) {
            tree_->sleep(time_to_sleep);
        } else {
            std::this_thread::sleep_for(time_to_sleep);
        }
        return true;
    }

    std::chrono::nanoseconds period() const {
        return std::chrono::duration_cast<std::chrono::nanoseconds>(period_);
    }

private:
    using Clock = std::chrono::steady_clock;
    std::chrono::milliseconds period_;
    Clock::time_point last_interval_;
    BT::Tree* tree_{nullptr};
};

}  // namespace behavior_tree
}  // namespace tasks
}  // namespace autonomy
