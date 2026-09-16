/*
 * Copyright 2026 The Openbot Authors
 *
 * Coordinates active controllers and trajectory execute / replace / cancel.
 */

#pragma once

#include <atomic>
#include <functional>
#include <memory>
#include <mutex>
#include <string>
#include <unordered_map>

#include "autonomy/manipulation/core/error_codes.hpp"
#include "autonomy/manipulation/execution/controller_manager.hpp"
#include "autonomy/manipulation/core/robot_model.hpp"

namespace autonomy {
namespace manipulation {
namespace execution {

/**
 * @brief Thin MoveIt-style trajectory execution manager over ControllerManager
 *        plugins (active controller, timeout, replace/preempt, deviation).
 */
class TrajectoryExecutionManager {
 public:
  /**
   * @brief Optional hook when desired vs actual joint state diverge.
   */
  using DeviationHook =
      std::function<void(const core::JointState& desired,
                         const core::JointState& actual)>;

  /** @brief Provides the latest measured joint state (e.g. JointStateSubscriber). */
  using StateProvider = std::function<core::JointState()>;

  /**
   * @brief Register a controller instance under @p id.
   * @param[in] id Lookup key (also becomes active if none set).
   * @param[in] controller Non-null controller plugin.
   */
  void RegisterController(const std::string& id,
                          std::shared_ptr<ControllerManager> controller);

  /**
   * @brief Select which registered controller Execute uses.
   * @param[in] id Previously registered controller id.
   */
  void SetActiveController(const std::string& id);

  /**
   * @brief Install a deviation callback (optional).
   * @param[in] hook Functor or empty to clear.
   */
  void SetDeviationHook(DeviationHook hook);

  /**
   * @brief Install measured-state provider used for deviation checks.
   * @param[in] provider Returns latest JointState; empty clears monitoring.
   */
  void SetStateProvider(StateProvider provider);

  /**
   * @brief Max per-joint |desired−actual| before failure (0 disables checks).
   * @param[in] tol Tolerance in rad or meters depending on joint type.
   */
  void SetDeviationTolerance(double tol) { deviation_tol_ = tol; }

  /**
   * @brief Maximum wall time allowed for a single Execute.
   * @param[in] seconds Timeout in seconds.
   */
  void SetTimeout(double seconds) { timeout_s_ = seconds; }

  /**
   * @brief Execute a trajectory on the active controller.
   * @param[in] trajectory Joint-space path.
   * @param[in] replace If true and already executing, cancel then start new.
   *                    If false and busy, return ErrorCode::kPreempted.
   * @return ErrorCode::kSuccess on success; control / preempt / timeout codes
   *         otherwise.
   */
  ErrorCode Execute(const core::RobotTrajectory& trajectory,
                    bool replace = false);

  /**
   * @brief Alias for Execute with replace=true (preempt current goal).
   * @param[in] trajectory Joint-space path that preempts the current goal.
   * @return Same codes as Execute.
   */
  ErrorCode ReplaceAndExecute(const core::RobotTrajectory& trajectory);

  /** @brief Cancel the active controller goal and clear the executing flag. */
  void Cancel();

  /** @brief Whether Execute currently holds the executing latch. */
  bool IsExecuting() const { return executing_.load(); }

 private:
  bool ExceedsDeviation(const core::JointState& desired,
                        const core::JointState& actual) const;

  std::unordered_map<std::string, std::shared_ptr<ControllerManager>>
      controllers_;
  std::string active_id_;
  DeviationHook deviation_hook_;
  StateProvider state_provider_;
  double deviation_tol_ = 0.0;
  double timeout_s_ = 30.0;
  std::atomic<bool> executing_{false};
  std::atomic<bool> cancel_{false};
  mutable std::mutex mutex_;
};

}  // namespace execution
}  // namespace manipulation
}  // namespace autonomy
