/*
 * Copyright 2026 The Openbot Authors
 */

#include "autonomy/manipulation/motion/execution/trajectory_execution_manager.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <thread>

#include "autonomy/common/logging.hpp"
#include "autonomy/manipulation/model/joint_state_utilities.hpp"

namespace autonomy {
namespace manipulation {
namespace execution {
namespace {

double WaypointDt(const automsgs::msgs::trajectory_msgs::JointTrajectory& trajectory, int index) {
  if (index < 0 || index >= trajectory.points_size()) {
    return 0.02;
  }
  const double t = GetTrajectoryPointTimeSeconds(trajectory, index);
  if (index == 0) {
    return std::max(0.0, t);
  }
  return std::max(0.0, t - GetTrajectoryPointTimeSeconds(trajectory, index - 1));
}

}  // namespace

void TrajectoryExecutionManager::RegisterController(
    const std::string& id, ControllerInterface::SharedPtr controller) {
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

void TrajectoryExecutionManager::SetEffortFeedforwardHook(
    EffortFeedforwardHook hook) {
  std::lock_guard<std::mutex> lock(mutex_);
  effort_hook_ = std::move(hook);
}

void TrajectoryExecutionManager::SetSceneValidityChecker(
    SceneValidityChecker checker) {
  std::lock_guard<std::mutex> lock(mutex_);
  scene_checker_ = std::move(checker);
}

bool TrajectoryExecutionManager::ExceedsDeviation(
    const automsgs::msgs::sensor_msgs::JointState& desired, const automsgs::msgs::sensor_msgs::JointState& actual) const {
  if (deviation_tol_ <= 0.0 || desired.position_size() == 0) {
    return false;
  }
  const int n = std::min(desired.position_size(), actual.position_size());
  if (n == 0) {
    return false;
  }
  for (int i = 0; i < n; ++i) {
    if (std::abs(desired.position(i) - actual.position(i)) > deviation_tol_) {
      return true;
    }
  }
  return false;
}

bool TrajectoryExecutionManager::ExceedsEffortDeviation(
    const automsgs::msgs::sensor_msgs::JointState& desired, const automsgs::msgs::sensor_msgs::JointState& actual) const {
  if (effort_tol_ <= 0.0 || desired.effort_size() == 0 ||
      actual.effort_size() == 0) {
    return false;
  }
  const int n = std::min(desired.effort_size(), actual.effort_size());
  for (int i = 0; i < n; ++i) {
    if (std::abs(desired.effort(i) - actual.effort(i)) > effort_tol_) {
      return true;
    }
  }
  return false;
}

ErrorCode TrajectoryExecutionManager::Execute(
    const automsgs::msgs::trajectory_msgs::JointTrajectory& trajectory, bool replace) {
  if (executing_.load()) {
    if (!replace) {
      return ErrorCode::PREEMPTED;
    }
    Cancel();
    for (int i = 0; i < 20 && executing_.load(); ++i) {
      std::this_thread::sleep_for(std::chrono::milliseconds(5));
    }
    if (executing_.load()) {
      executing_.store(false);
    }
  }

  ControllerInterface::SharedPtr controller;
  StateProvider provider;
  DeviationHook hook;
  EffortFeedforwardHook effort_hook;
  SceneValidityChecker scene_checker;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    const auto it = controllers_.find(active_id_);
    if (it == controllers_.end() || !it->second) {
      return ErrorCode::CONTROL_FAILED;
    }
    controller = it->second;
    provider = state_provider_;
    hook = deviation_hook_;
    effort_hook = effort_hook_;
    scene_checker = scene_checker_;
  }

  cancel_.store(false);
  executing_.store(true);
  const auto start = std::chrono::steady_clock::now();

  auto apply_effort = [&](const automsgs::msgs::sensor_msgs::JointState& desired) {
    if (effort_hook && desired.effort_size() > 0) {
      effort_hook(desired);
    }
  };

  auto check_scene = [&](const automsgs::msgs::sensor_msgs::JointState& desired) -> bool {
    if (!scene_checker) {
      return true;
    }
    return scene_checker(desired);
  };

  auto check_dev = [&](const automsgs::msgs::sensor_msgs::JointState& desired) -> bool {
    if (!provider) {
      return true;
    }
    const automsgs::msgs::sensor_msgs::JointState actual = provider();
    if (ExceedsDeviation(desired, actual) ||
        ExceedsEffortDeviation(desired, actual)) {
      if (hook) {
        hook(desired, actual);
      }
      return false;
    }
    return true;
  };

  bool ok = true;
  if ((provider && (deviation_tol_ > 0.0 || effort_tol_ > 0.0)) ||
      scene_checker || trajectory.points_size() > 1) {
    const bool step_mode =
        (provider && (deviation_tol_ > 0.0 || effort_tol_ > 0.0)) ||
        static_cast<bool>(scene_checker);
    if (step_mode && trajectory.points_size() > 1) {
      for (int i = 0; i < trajectory.points_size(); ++i) {
        if (cancel_.load()) {
          executing_.store(false);
          return ErrorCode::PREEMPTED;
        }
        const automsgs::msgs::sensor_msgs::JointState desired =
            MakeJointStateFromPoint(trajectory, i);
        if (!check_scene(desired)) {
          AWARN << "TrajectoryExecutionManager: scene invalid at waypoint " << i;
          Cancel();
          executing_.store(false);
          return ErrorCode::INVALID_MOTION_PLAN;
        }
        apply_effort(desired);
        automsgs::msgs::trajectory_msgs::JointTrajectory step;
        AddTrajectoryPoint(&step, desired, GetTrajectoryPointTimeSeconds(trajectory, i));
        if (!controller->FollowJointTrajectory(step)) {
          ok = false;
          break;
        }
        const double dt = WaypointDt(trajectory, i);
        if (dt > 0.0) {
          std::this_thread::sleep_for(
              std::chrono::duration<double>(std::min(dt, 0.2)));
        }
        if (!check_dev(desired)) {
          Cancel();
          executing_.store(false);
          return ErrorCode::CONTROL_FAILED;
        }
      }
    } else {
      if (trajectory.points_size() > 0) {
        const automsgs::msgs::sensor_msgs::JointState front =
            MakeJointStateFromPoint(trajectory, 0);
        if (!check_scene(front)) {
          executing_.store(false);
          return ErrorCode::INVALID_MOTION_PLAN;
        }
        apply_effort(front);
        if (!check_dev(front)) {
          executing_.store(false);
          return ErrorCode::CONTROL_FAILED;
        }
      }
      ok = controller->FollowJointTrajectory(trajectory);
      if (ok && trajectory.points_size() > 0) {
        const automsgs::msgs::sensor_msgs::JointState back = MakeJointStateFromPoint(
            trajectory, trajectory.points_size() - 1);
        if (!check_scene(back)) {
          executing_.store(false);
          return ErrorCode::INVALID_MOTION_PLAN;
        }
        if (!check_dev(back)) {
          executing_.store(false);
          return ErrorCode::CONTROL_FAILED;
        }
      }
    }
  } else {
    if (trajectory.points_size() > 0) {
      apply_effort(MakeJointStateFromPoint(trajectory, 0));
    }
    ok = controller->FollowJointTrajectory(trajectory);
  }

  executing_.store(false);

  if (cancel_.load()) {
    return ErrorCode::PREEMPTED;
  }
  const double elapsed =
      std::chrono::duration<double>(std::chrono::steady_clock::now() - start)
          .count();
  if (elapsed > timeout_s_) {
    AWARN << "TrajectoryExecutionManager: exceeded timeout";
    return ErrorCode::TIMED_OUT;
  }
  return ok ? ErrorCode::SUCCESS : ErrorCode::CONTROL_FAILED;
}

ErrorCode TrajectoryExecutionManager::ReplaceAndExecute(
    const automsgs::msgs::trajectory_msgs::JointTrajectory& trajectory) {
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
