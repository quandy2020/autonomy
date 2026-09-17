// Copyright 2017 Open Source Robotics Foundation, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

// Ported into autonomy::control::tools::controllers
// ROS-free joint-trajectory tracker state machine (no action server).

#ifndef AUTONOMY_CONTROL_TOOLS_CONTROLLERS_JOINT_TRAJECTORY__TRAJECTORY_TRACKER_HPP_
#define AUTONOMY_CONTROL_TOOLS_CONTROLLERS_JOINT_TRAJECTORY__TRAJECTORY_TRACKER_HPP_

#include <algorithm>
#include <cmath>
#include <memory>
#include <vector>

#include "autonomy/control/tools/controllers/joint_trajectory/tolerances.hpp"
#include "autonomy/control/tools/controllers/joint_trajectory/trajectory.hpp"
#include "autonomy/control/tools/controllers/joint_trajectory/trajectory_types.hpp"

namespace autonomy {
namespace control {
namespace tools {
namespace controllers {
namespace joint_trajectory
{

enum class TrackerState
{
  Idle,
  Tracking,
  Hold,
  Stopping,
  Succeeded,
  Aborted
};

/**
 * \brief Sample a `Trajectory`, hold the last point, stop with deceleration, and
 * check goal tolerances — without FollowJointTrajectory action plumbing.
 */
class TrajectoryTracker
{
public:
  TrajectoryTracker() = default;

  void set_tolerances(const SegmentTolerances & tolerances) { tolerances_ = tolerances; }

  const SegmentTolerances & tolerances() const { return tolerances_; }

  /**
   * \brief Install a new trajectory and enter Tracking.
   * \param trajectory   Shared joint trajectory (may be empty → Idle)
   * \param start_time_sec Absolute time [s] used as trajectory start
   */
  void set_trajectory(std::shared_ptr<JointTrajectory> trajectory, double start_time_sec)
  {
    if (!trajectory || trajectory->points.empty())
    {
      trajectory_.reset();
      traj_wrapper_.reset();
      state_ = TrackerState::Idle;
      return;
    }

    // Stamp absolute start onto the message so Trajectory::sample resolves correctly.
    trajectory->header_stamp = start_time_sec;
    trajectory_ = std::move(trajectory);
    traj_wrapper_ = std::make_unique<Trajectory>(trajectory_);
    start_time_sec_ = start_time_sec;
    state_ = TrackerState::Tracking;
    hold_point_valid_ = false;
  }

  TrackerState state() const { return state_; }

  /** \brief Request a controlled stop → Stopping, then Idle / Aborted. */
  void stop()
  {
    if (state_ == TrackerState::Idle || state_ == TrackerState::Succeeded ||
        state_ == TrackerState::Aborted)
    {
      return;
    }
    state_ = TrackerState::Stopping;
  }

  void reset()
  {
    trajectory_.reset();
    traj_wrapper_.reset();
    state_ = TrackerState::Idle;
    hold_point_valid_ = false;
  }

  /**
   * \brief One control step.
   *
   * \param time_sec   Absolute time [s]
   * \param current    Measured joint state
   * \param desired    Output command sample
   * \return Current tracker state after the update
   */
  TrackerState update(
    double time_sec, const TrajectoryPoint & current, TrajectoryPoint & desired)
  {
    last_time_sec_ = time_sec;

    switch (state_)
    {
      case TrackerState::Idle:
        desired = current;
        zero_derivatives(desired);
        break;

      case TrackerState::Tracking:
        update_tracking(time_sec, current, desired);
        break;

      case TrackerState::Hold:
        if (hold_point_valid_)
        {
          desired = hold_point_;
        }
        else
        {
          desired = current;
          zero_derivatives(desired);
        }
        if (within_goal_tolerance(current, desired))
        {
          state_ = TrackerState::Succeeded;
        }
        break;

      case TrackerState::Stopping:
        update_stopping(current, desired);
        break;

      case TrackerState::Succeeded:
      case TrackerState::Aborted:
        if (hold_point_valid_)
        {
          desired = hold_point_;
        }
        else
        {
          desired = current;
          zero_derivatives(desired);
        }
        break;
    }
    return state_;
  }

  /** Max |Δv|/dt used while Stopping (per joint). Default 1.0 rad/s². */
  void set_stop_deceleration(double decel) { stop_deceleration_ = std::max(0.0, decel); }

  double stop_deceleration() const { return stop_deceleration_; }

private:
  void update_tracking(
    double time_sec, const TrajectoryPoint & current, TrajectoryPoint & desired)
  {
    if (!traj_wrapper_ || !trajectory_)
    {
      state_ = TrackerState::Idle;
      desired = current;
      return;
    }

    TrajectoryPointConstIter start_it, end_it;
    const bool ok = traj_wrapper_->sample(
      time_sec, interpolation_methods::DEFAULT_INTERPOLATION, desired, start_it, end_it);

    if (!ok)
    {
      state_ = TrackerState::Aborted;
      desired = current;
      zero_derivatives(desired);
      hold_point_ = desired;
      hold_point_valid_ = true;
      return;
    }

    desired.time_from_start = time_sec - start_time_sec_;

    const bool before_last = end_it != traj_wrapper_->end();
    if (!before_last)
    {
      // Past (or at) the final segment: hold last sample and check success.
      hold_point_ = desired;
      hold_point_valid_ = true;
      zero_derivatives(hold_point_);
      state_ = TrackerState::Hold;
      if (within_goal_tolerance(current, desired))
      {
        state_ = TrackerState::Succeeded;
      }
      return;
    }

    // Path tolerance while moving (optional — skip if state_tolerance empty / zero).
    if (!tolerances_.state_tolerance.empty())
    {
      TrajectoryPoint error = compute_error(current, desired);
      for (size_t i = 0; i < tolerances_.state_tolerance.size() && i < error.positions.size();
           ++i)
      {
        if (!check_state_tolerance_per_joint(error, i, tolerances_.state_tolerance[i]))
        {
          state_ = TrackerState::Aborted;
          hold_point_ = current;
          zero_derivatives(hold_point_);
          hold_point_valid_ = true;
          desired = hold_point_;
          return;
        }
      }
    }
  }

  void update_stopping(const TrajectoryPoint & current, TrajectoryPoint & desired)
  {
    desired = current;
    const size_t n = current.positions.size();
    if (desired.velocities.size() != n)
    {
      desired.velocities.assign(n, 0.0);
    }
    if (desired.accelerations.size() != n)
    {
      desired.accelerations.assign(n, 0.0);
    }

    // Decelerate commanded velocities toward zero; hold measured position.
    constexpr double dt_fallback = 0.01;
    double step = dt_fallback;
    if (have_prev_time_ && last_time_sec_ > prev_time_sec_)
    {
      step = last_time_sec_ - prev_time_sec_;
    }
    else if (last_dt_ > 0.0)
    {
      step = last_dt_;
    }
    prev_time_sec_ = last_time_sec_;
    have_prev_time_ = true;
    last_dt_ = step;

    bool all_stopped = true;
    for (size_t i = 0; i < n; ++i)
    {
      double v = current.velocities.size() > i ? current.velocities[i] : 0.0;
      const double max_dv = stop_deceleration_ * step;
      if (std::fabs(v) <= max_dv)
      {
        v = 0.0;
      }
      else
      {
        v -= std::copysign(max_dv, v);
        all_stopped = false;
      }
      desired.velocities[i] = v;
      desired.accelerations[i] = 0.0;
    }

    hold_point_ = desired;
    zero_derivatives(hold_point_);
    hold_point_.positions = current.positions;
    hold_point_valid_ = true;

    if (all_stopped)
    {
      desired = hold_point_;
      state_ = TrackerState::Aborted;
    }
  }

  bool within_goal_tolerance(const TrajectoryPoint & current, const TrajectoryPoint & desired) const
  {
    if (tolerances_.goal_state_tolerance.empty())
    {
      // Default: position-only with a small absolute tolerance if none configured.
      const size_t n = std::min(current.positions.size(), desired.positions.size());
      for (size_t i = 0; i < n; ++i)
      {
        if (std::fabs(current.positions[i] - desired.positions[i]) > 1e-3)
        {
          return false;
        }
      }
      return n > 0;
    }

    TrajectoryPoint error = compute_error(current, desired);
    for (size_t i = 0; i < tolerances_.goal_state_tolerance.size() && i < error.positions.size();
         ++i)
    {
      if (!check_state_tolerance_per_joint(error, i, tolerances_.goal_state_tolerance[i]))
      {
        return false;
      }
    }
    return true;
  }

  static TrajectoryPoint compute_error(
    const TrajectoryPoint & current, const TrajectoryPoint & desired)
  {
    TrajectoryPoint error;
    const size_t n = std::min(current.positions.size(), desired.positions.size());
    error.positions.resize(n);
    for (size_t i = 0; i < n; ++i)
    {
      error.positions[i] = desired.positions[i] - current.positions[i];
    }
    if (!current.velocities.empty() && !desired.velocities.empty())
    {
      const size_t nv = std::min(current.velocities.size(), desired.velocities.size());
      error.velocities.resize(nv);
      for (size_t i = 0; i < nv; ++i)
      {
        error.velocities[i] = desired.velocities[i] - current.velocities[i];
      }
    }
    if (!current.accelerations.empty() && !desired.accelerations.empty())
    {
      const size_t na = std::min(current.accelerations.size(), desired.accelerations.size());
      error.accelerations.resize(na);
      for (size_t i = 0; i < na; ++i)
      {
        error.accelerations[i] = desired.accelerations[i] - current.accelerations[i];
      }
    }
    return error;
  }

  static void zero_derivatives(TrajectoryPoint & pt)
  {
    if (!pt.velocities.empty())
    {
      std::fill(pt.velocities.begin(), pt.velocities.end(), 0.0);
    }
    if (!pt.accelerations.empty())
    {
      std::fill(pt.accelerations.begin(), pt.accelerations.end(), 0.0);
    }
  }

  std::shared_ptr<JointTrajectory> trajectory_;
  std::unique_ptr<Trajectory> traj_wrapper_;
  SegmentTolerances tolerances_;
  TrackerState state_{TrackerState::Idle};
  double start_time_sec_{0.0};
  double last_time_sec_{0.0};
  double prev_time_sec_{0.0};
  bool have_prev_time_{false};
  double last_dt_{0.0};
  double stop_deceleration_{1.0};
  TrajectoryPoint hold_point_;
  bool hold_point_valid_{false};
};

}  // namespace joint_trajectory
}  // namespace controllers
}  // namespace tools
}  // namespace control
}  // namespace autonomy

#endif  // AUTONOMY_CONTROL_TOOLS_CONTROLLERS_JOINT_TRAJECTORY__TRAJECTORY_TRACKER_HPP_
