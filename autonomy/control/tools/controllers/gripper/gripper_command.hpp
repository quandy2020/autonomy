// Copyright 2014, SRI International
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
// Gripper command state machine (no action server / hardware interfaces).

#ifndef AUTONOMY_CONTROL_TOOLS_CONTROLLERS_GRIPPER__GRIPPER_COMMAND_HPP_
#define AUTONOMY_CONTROL_TOOLS_CONTROLLERS_GRIPPER__GRIPPER_COMMAND_HPP_

#include <cmath>

namespace autonomy {
namespace control {
namespace tools {
namespace controllers {
namespace gripper
{

enum class GripperStatus
{
  Idle,
  Moving,
  Succeeded,
  Stalled
};

struct GripperGoal
{
  double position{0.0};
  double max_effort{0.0};
};

struct GripperCommandOutput
{
  double command_position{0.0};
  double command_effort{0.0};
  GripperStatus status{GripperStatus::Idle};
};

/**
 * \brief Single-joint gripper goal tracker mirroring upstream
 * `check_for_success` / command update logic without ROS actions.
 *
 * Succeeded when `|error| < goal_tolerance`.
 * Stalled when `|velocity| < stall_velocity_threshold` for `stall_timeout_sec`
 * while not yet at the goal.
 */
class GripperCommand
{
public:
  GripperCommand() = default;

  void set_goal_tolerance(double tol) { goal_tolerance_ = tol; }
  void set_stall_velocity_threshold(double th) { stall_velocity_threshold_ = th; }
  void set_stall_timeout(double sec) { stall_timeout_sec_ = sec; }
  /** If true, stall transitions to Succeeded (upstream allow_stalling). */
  void set_allow_stalling(bool allow) { allow_stalling_ = allow; }

  double goal_tolerance() const { return goal_tolerance_; }
  double stall_velocity_threshold() const { return stall_velocity_threshold_; }
  double stall_timeout() const { return stall_timeout_sec_; }
  bool allow_stalling() const { return allow_stalling_; }

  GripperStatus status() const { return status_; }
  const GripperGoal & goal() const { return goal_; }

  /** \brief Accept a new goal and enter Moving. */
  void set_goal(const GripperGoal & goal, double time_sec)
  {
    goal_ = goal;
    status_ = GripperStatus::Moving;
    last_movement_time_sec_ = time_sec;
    has_goal_ = true;
  }

  void set_goal(double position, double max_effort, double time_sec)
  {
    set_goal(GripperGoal{position, max_effort}, time_sec);
  }

  /** \brief Clear active goal → Idle; command holds `current_pos`. */
  void cancel(double current_pos)
  {
    has_goal_ = false;
    status_ = GripperStatus::Idle;
    goal_.position = current_pos;
  }

  void reset()
  {
    has_goal_ = false;
    status_ = GripperStatus::Idle;
    goal_ = {};
  }

  /**
   * \brief One update step.
   *
   * While Moving / after terminal states, `command_position` is the goal
   * position and `command_effort` is `max_effort` (caller applies to HW).
   */
  GripperCommandOutput update(double time_sec, double current_pos, double current_vel)
  {
    GripperCommandOutput out;
    out.command_position = goal_.position;
    out.command_effort = goal_.max_effort;
    out.status = status_;

    if (!has_goal_ || status_ == GripperStatus::Idle)
    {
      out.command_position = current_pos;
      out.status = GripperStatus::Idle;
      status_ = GripperStatus::Idle;
      return out;
    }

    if (status_ == GripperStatus::Succeeded || status_ == GripperStatus::Stalled)
    {
      out.status = status_;
      return out;
    }

    // Moving
    const double error = goal_.position - current_pos;
    if (std::fabs(error) < goal_tolerance_)
    {
      status_ = GripperStatus::Succeeded;
      out.status = status_;
      return out;
    }

    if (std::fabs(current_vel) > stall_velocity_threshold_)
    {
      last_movement_time_sec_ = time_sec;
    }
    else if ((time_sec - last_movement_time_sec_) > stall_timeout_sec_)
    {
      status_ = allow_stalling_ ? GripperStatus::Succeeded : GripperStatus::Stalled;
      out.status = status_;
      return out;
    }

    out.status = GripperStatus::Moving;
    status_ = GripperStatus::Moving;
    return out;
  }

private:
  GripperGoal goal_;
  GripperStatus status_{GripperStatus::Idle};
  bool has_goal_{false};
  double last_movement_time_sec_{0.0};
  double goal_tolerance_{0.01};
  double stall_velocity_threshold_{1e-4};
  double stall_timeout_sec_{1.0};
  bool allow_stalling_{false};
};

}  // namespace gripper
}  // namespace controllers
}  // namespace tools
}  // namespace control
}  // namespace autonomy

#endif  // AUTONOMY_CONTROL_TOOLS_CONTROLLERS_GRIPPER__GRIPPER_COMMAND_HPP_
