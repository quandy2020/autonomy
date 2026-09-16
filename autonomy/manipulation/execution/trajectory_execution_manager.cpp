/*
 * Copyright 2026 The Openbot Authors
 */

#include "autonomy/manipulation/execution/trajectory_execution_manager.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <thread>

#include "autonomy/common/logging.hpp"

namespace autonomy {
namespace manipulation {
namespace execution {
namespace {

double WaypointDt(const core::RobotTrajectory& trajectory, std::size_t index) {
  if (trajectory.time_from_start.size() <= index) {
    return 0.02;
  }
  if (index == 0) {
    return std::max(0.0, trajectory.time_from_start[0]);
  }
  return std::max(
      0.0, trajectory.time_from_start[index] - trajectory.time_from_start[index - 1]);
}

}  // namespace

void TrajectoryExecutionManager::RegisterController(
    const std::string& id, std::shared_ptr<ControllerManager> controller) {
  std::lock_guard<std::mutex> lock(mutex_);
  controllers_[id] = std::move(controller);
  if (active_id_.empty()) {
    active_id_ = id;
  }
}

void TrajectoryExecutionManager::SetActiveController(const std::string& id) {
  std::lock_guard<std::mutex> lock(mutex_);
  active_id_ = id;
}

void TrajectoryExecutionManager::SetDeviationHook(DeviationHook hook) {
  std::lock_guard<std::mutex> lock(mutex_);
  deviation_hook_ = std::move(hook);
}

void TrajectoryExecutionManager::SetStateProvider(StateProvider provider) {
  std::lock_guard<std::mutex> lock(mutex_);
  state_provider_ = std::move(provider);
}

bool TrajectoryExecutionManager::ExceedsDeviation(
    const core::JointState& desired, const core::JointState& actual) const {
  if (deviation_tol_ <= 0.0 || desired.positions.empty()) {
    return false;
  }
  const std::size_t n =
      std::min(desired.positions.size(), actual.positions.size());
  if (n == 0) {
    return false;
  }
  for (std::size_t i = 0; i < n; ++i) {
    if (std::abs(desired.positions[i] - actual.positions[i]) > deviation_tol_) {
      return true;
    }
  }
  return false;
}

ErrorCode TrajectoryExecutionManager::Execute(
    const core::RobotTrajectory& trajectory, bool replace) {
  if (executing_.load()) {
    if (!replace) {
      return ErrorCode::kPreempted;
    }
    Cancel();
    for (int i = 0; i < 20 && executing_.load(); ++i) {
      std::this_thread::sleep_for(std::chrono::milliseconds(5));
    }
    if (executing_.load()) {
      executing_.store(false);
    }
  }

  std::shared_ptr<ControllerManager> controller;
  StateProvider provider;
  DeviationHook hook;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    const auto it = controllers_.find(active_id_);
    if (it == controllers_.end() || !it->second) {
      return ErrorCode::kControlFailed;
    }
    controller = it->second;
    provider = state_provider_;
    hook = deviation_hook_;
  }

  cancel_.store(false);
  executing_.store(true);
  const auto start = std::chrono::steady_clock::now();

  auto check_dev = [&](const core::JointState& desired) -> bool {
    if (!provider || deviation_tol_ <= 0.0) {
      return true;
    }
    const core::JointState actual = provider();
    if (!ExceedsDeviation(desired, actual)) {
      return true;
    }
    if (hook) {
      hook(desired, actual);
    }
    return false;
  };

  bool ok = true;
  if (provider && deviation_tol_ > 0.0 && trajectory.waypoints.size() > 1) {
    for (std::size_t i = 0; i < trajectory.waypoints.size(); ++i) {
      if (cancel_.load()) {
        executing_.store(false);
        return ErrorCode::kPreempted;
      }
      core::RobotTrajectory step;
      step.waypoints.push_back(trajectory.waypoints[i]);
      if (i < trajectory.time_from_start.size()) {
        step.time_from_start.push_back(trajectory.time_from_start[i]);
      }
      if (!controller->Execute(step)) {
        ok = false;
        break;
      }
      const double dt = WaypointDt(trajectory, i);
      if (dt > 0.0) {
        std::this_thread::sleep_for(
            std::chrono::duration<double>(std::min(dt, 0.2)));
      }
      if (!check_dev(trajectory.waypoints[i])) {
        Cancel();
        executing_.store(false);
        return ErrorCode::kControlFailed;
      }
    }
  } else {
    if (!trajectory.waypoints.empty() && !check_dev(trajectory.waypoints.front())) {
      executing_.store(false);
      return ErrorCode::kControlFailed;
    }
    ok = controller->Execute(trajectory);
    if (ok && !trajectory.waypoints.empty() &&
        !check_dev(trajectory.waypoints.back())) {
      executing_.store(false);
      return ErrorCode::kControlFailed;
    }
  }

  executing_.store(false);

  if (cancel_.load()) {
    return ErrorCode::kPreempted;
  }
  const double elapsed =
      std::chrono::duration<double>(std::chrono::steady_clock::now() - start)
          .count();
  if (elapsed > timeout_s_) {
    AWARN << "TrajectoryExecutionManager: exceeded timeout";
    return ErrorCode::kTimedOut;
  }
  return ok ? ErrorCode::kSuccess : ErrorCode::kControlFailed;
}

ErrorCode TrajectoryExecutionManager::ReplaceAndExecute(
    const core::RobotTrajectory& trajectory) {
  return Execute(trajectory, true);
}

void TrajectoryExecutionManager::Cancel() {
  cancel_.store(true);
  std::lock_guard<std::mutex> lock(mutex_);
  const auto it = controllers_.find(active_id_);
  if (it != controllers_.end() && it->second) {
    it->second->Cancel();
  }
}

}  // namespace execution
}  // namespace manipulation
}  // namespace autonomy
