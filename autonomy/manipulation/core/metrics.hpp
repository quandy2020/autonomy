/*
 * Copyright 2026 The Openbot Authors
 */

#pragma once

#include <atomic>
#include <chrono>
#include <cstdint>

#include "autonomy/common/logging.hpp"

namespace autonomy {
namespace manipulation {
namespace metrics {

/**
 * @brief Process-wide counters for planning / IK success and latency.
 *
 * Singleton accessor via @ref Instance; intended for lightweight logging and
 * diagnostics, not cross-process telemetry.
 */
class ManipulationMetrics {
 public:
  /** @brief Process-wide singleton instance. */
  static ManipulationMetrics& Instance() {
    static ManipulationMetrics m;
    return m;
  }

  /** @brief Mark the start of a planning attempt (starts wall-time timer). */
  void OnPlanStart() {
    plan_start_ = std::chrono::steady_clock::now();
  }

  /**
   * @brief Record plan completion, update counters, and log a summary line.
   * @param[in] success Whether planning succeeded.
   */
  void OnPlanEnd(bool success) {
    const auto dt = std::chrono::duration<double>(
                        std::chrono::steady_clock::now() - plan_start_)
                        .count();
    ++plan_count_;
    if (success) {
      ++plan_success_;
    }
    last_plan_s_ = dt;
    AINFO << "metrics plan success=" << success << " dt_s=" << dt
          << " total=" << plan_count_ << " ok=" << plan_success_;
  }

  /**
   * @brief Record an IK solve attempt.
   * @param[in] success Whether IK succeeded.
   */
  void OnIk(bool success) {
    ++ik_count_;
    if (success) {
      ++ik_success_;
    }
  }

  /** @brief Total planning attempts. */
  std::uint64_t plan_count() const { return plan_count_.load(); }

  /** @brief Successful planning attempts. */
  std::uint64_t plan_success() const { return plan_success_.load(); }

  /** @brief Total IK attempts. */
  std::uint64_t ik_count() const { return ik_count_.load(); }

  /** @brief Successful IK attempts. */
  std::uint64_t ik_success() const { return ik_success_.load(); }

  /** @brief Wall time of the last completed plan, in seconds. */
  double last_plan_s() const { return last_plan_s_; }

 private:
  std::atomic<std::uint64_t> plan_count_{0};
  std::atomic<std::uint64_t> plan_success_{0};
  std::atomic<std::uint64_t> ik_count_{0};
  std::atomic<std::uint64_t> ik_success_{0};
  std::chrono::steady_clock::time_point plan_start_{};
  double last_plan_s_ = 0.0;
};

}  // namespace metrics
}  // namespace manipulation
}  // namespace autonomy
