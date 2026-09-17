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
 * @brief Process-wide counters for planning / inverse-kinematics success and
 * latency.
 *
 * Singleton accessor via @ref Instance; intended for lightweight logging and
 * diagnostics, not cross-process telemetry.
 */
class ManipulationMetrics {
 public:
  /** @brief Process-wide singleton instance. */
  static ManipulationMetrics& Instance() {
    static ManipulationMetrics instance;
    return instance;
  }

  /** @brief Mark the start of a planning attempt (starts wall-time timer). */
  void RecordPlanAttemptStart() {
    plan_attempt_start_time_ = std::chrono::steady_clock::now();
  }

  /**
   * @brief Record plan completion, update counters, and log a summary line.
   * @param[in] success Whether planning succeeded.
   */
  void RecordPlanAttemptEnd(bool success) {
    const auto duration_seconds = std::chrono::duration<double>(
                                      std::chrono::steady_clock::now() -
                                      plan_attempt_start_time_)
                                      .count();
    ++plan_attempt_count_;
    if (success) {
      ++plan_success_count_;
    }
    last_plan_duration_seconds_ = duration_seconds;
    AINFO << "metrics plan success=" << success
          << " duration_s=" << duration_seconds
          << " total=" << plan_attempt_count_ << " ok=" << plan_success_count_;
  }

  /**
   * @brief Record an inverse-kinematics solve attempt.
   * @param[in] success Whether inverse kinematics succeeded.
   */
  void RecordInverseKinematicsAttempt(bool success) {
    ++inverse_kinematics_attempt_count_;
    if (success) {
      ++inverse_kinematics_success_count_;
    }
  }

  /** @brief Total planning attempts. */
  std::uint64_t GetPlanAttemptCount() const {
    return plan_attempt_count_.load();
  }

  /** @brief Successful planning attempts. */
  std::uint64_t GetPlanSuccessCount() const {
    return plan_success_count_.load();
  }

  /** @brief Total inverse-kinematics attempts. */
  std::uint64_t GetInverseKinematicsAttemptCount() const {
    return inverse_kinematics_attempt_count_.load();
  }

  /** @brief Successful inverse-kinematics attempts. */
  std::uint64_t GetInverseKinematicsSuccessCount() const {
    return inverse_kinematics_success_count_.load();
  }

  /** @brief Wall time of the last completed plan, in seconds. */
  double GetLastPlanDurationSeconds() const {
    return last_plan_duration_seconds_;
  }

 private:
  std::atomic<std::uint64_t> plan_attempt_count_{0};
  std::atomic<std::uint64_t> plan_success_count_{0};
  std::atomic<std::uint64_t> inverse_kinematics_attempt_count_{0};
  std::atomic<std::uint64_t> inverse_kinematics_success_count_{0};
  std::chrono::steady_clock::time_point plan_attempt_start_time_{};
  double last_plan_duration_seconds_ = 0.0;
};

}  // namespace metrics
}  // namespace manipulation
}  // namespace autonomy
