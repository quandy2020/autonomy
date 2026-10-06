/*
 * Copyright 2025 The Openbot Authors (duyongquan)
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

/**
 * @file sando_planner.hpp
 * @brief One-cycle ground SANDO loop from a costmap to a sampled setpoint.
 *
 * A cycle projects a horizon goal, searches a heat-weighted path, builds a
 * time-layered polyhedral corridor, and solves a min-jerk MIQP. The committed
 * prefix of the previous sample sequence is kept, and the new samples are
 * appended after that prefix. Yaw is filtered separately from the polynomial.
 */

#pragma once

#include <chrono>
#include <deque>
#include <string>
#include <vector>

#include "autonomy/control/controller/sando_controller/safe_corridor.hpp"
#include "autonomy/control/controller/sando_controller/occupancy_grid.hpp"
#include "autonomy/control/controller/sando_controller/hover_monitor.hpp"
#include "autonomy/control/controller/sando_controller/obstacle_tracker.hpp"
#include "autonomy/control/controller/sando_controller/geometric_path.hpp"
#include "autonomy/control/controller/sando_controller/path_searcher.hpp"
#include "autonomy/control/controller/sando_controller/trajectory_optimizer.hpp"
#include "autonomy/control/proto/sando_controller.pb.h"
#include "autonomy/map/costmap_2d/costmap_2d.hpp"

namespace autonomy {
namespace control {
namespace controller {
namespace sando_controller {

/**
 * @class SandoPlanner
 * @brief Online planar SANDO planner.
 *
 * The public surface is deliberately small: configure, reset, accept a
 * terminal goal and a costmap, then ask for one command. Search, corridor
 * generation, the factor window, and yaw live in the private methods.
 */
class SandoPlanner {
 public:
  /**
   * @brief Copy options, fill defaults, and reset internal state.
   * @param options Proto options. Empty strings and non-positive limits are replaced by ApplyDefaults.
   */
  void Configure(const proto::SandoControllerOptions& options);

  /**
   * @brief Drop the committed plan, the path, the tracker, and the yaw timer.
   *
   * The terminal goal is kept when goal_ready_ is already true so a later
   * cycle can replan to the same pose. Status returns to yawing.
   */
  void Reset();

  /**
   * @brief Set the user goal. Hover evasion writes a different working goal and does not change this pose.
   * @param x Terminal x, meters, world frame.
   * @param y Terminal y, meters, world frame.
   * @param yaw Desired heading at the terminal pose, radians.
   */
  void SetTerminalGoal(double x, double y, double yaw);

  /**
   * @brief Copy the costmap into the occupancy grid, track clusters, and rebuild the heat field.
   * @param costmap Local costmap. The caller holds its mutex.
   * @param now Clock time, seconds, used to age obstacles and to predict their centers.
   * @param robot_x Robot x, meters. Seeds the flood fill and the heat focus.
   * @param robot_y Robot y, meters.
   */
  void IngestCostmap(const map::costmap_2d::Costmap2D& costmap, double now, double robot_x,
                 double robot_y);

  /**
   * @brief Forward one external mover to the tracker.
   * @param x Center x, meters.
   * @param y Center y, meters.
   * @param velocity_x World x velocity, m/s.
   * @param velocity_y World y velocity, m/s.
   * @param radius Circumscribed radius, meters.
   */
  void AddDynamicObstacle(double x, double y, double velocity_x, double velocity_y, double radius);

  /**
   * @brief Produce the next world-frame setpoint, replanning when NeedsNewPlan says so.
   *
   * The returned command is the front of the sample queue after one pop.
   * Position is world-frame. Yaw and yaw rate are filled by ComputeDesiredYaw
   * and are not part of the polynomial.
   *
   * @param robot Measured state. Velocity must already be in the world frame.
   * @param now Clock time, seconds.
   * @param command Output setpoint. Required.
   * @param message Optional failure text. May be null.
   * @return False when there is no goal, no map, or the replan produced no usable samples.
   */
  bool ComputeCommand(const State& robot, double now, State* command, std::string* message);

  /**
   * @brief True when the status is kGoalReached.
   *
   * This is the plan-tail condition, not a distance test on the robot. Hover
   * avoidance does not set this status.
   */
  bool IsTerminalGoalReached() const { return status_ == Status::kGoalReached; }

  /**
   * @brief Planar distance from the last robot state to the terminal goal, meters.
   */
  double GetGoalDistance() const { return goal_distance_; }

  /**
   * @brief Wrapped yaw error between the terminal heading and the robot, radians.
   */
  double GetGoalYawError() const { return goal_yaw_error_; }

 private:
  /**
   * @brief Decide whether this cycle should throw away the tail and replan.
   *
   * kGoalSeen stops replanning once the last committed sample is inside the
   * goal radius of the terminal pose. Hover statuses force a replan. Yawing
   * and goal-reached do not. A robot that is already inside the radius and
   * slower than 0.1 m/s also skips the replan, except while yawing.
   *
   * @param robot Measured state.
   * @return True when ReplanTrajectory should run.
   */
  bool NeedsNewPlan(const State& robot) const;

  /**
   * @brief Search, decompose, and solve from the committed state A.
   *
   * The factor window tries means around last_factor_ in steps of factor_step,
   * smaller factors first, and keeps the first trajectory that passes
   * IsWithinLimits. One over-limit trial is retained as a fallback.
   * The sample queue keeps the first `commit` samples and appends the new
   * tail after dropping the duplicate of A.
   *
   * @param robot Measured state, used when the committed prefix has drifted more than 0.45 m.
   * @param now Clock time, seconds. Stamps the replan and the computation-time filter.
   * @return True when at least a fallback trajectory was accepted.
   */
  bool ReplanTrajectory(const State& robot, double now);

  /**
   * @brief Write yaw and yaw rate into the command without changing its linear velocity.
   *
   * Spinning takes priority once failure_count_ exceeds the threshold and
   * hover is off. A plan shorter than five samples holds the previous yaw and
   * zeros the linear velocity.
   * Yawing converges on the measured robot yaw and steps the command from
   * previous_yaw_. Hover inside 0.3 m holds yaw. Travel below 0.01 m/s holds yaw.
   *
   * @param robot Measured state. Its yaw is the convergence reference, not the command source.
   * @param command Setpoint whose yaw and yaw_rate are overwritten.
   */
  void ComputeDesiredYaw(const State& robot, State* command);

  /**
   * @brief Place the search goal on the segment toward the terminal pose, at most one horizon away.
   * @param from State the horizon is measured from, usually the committed state A.
   * @param goal_x Output goal x, meters. Required.
   * @param goal_y Output goal y, meters. Required.
   */
  void ProjectHorizonGoal(const State& from, double* goal_x, double* goal_y) const;

  proto::SandoControllerOptions options_;  ///< Active options, after defaults.
  OccupancyGrid grid_;                  ///< Inflated occupancy, heat, and obstacle queries for this cycle.
  bool map_ready_{false};               ///< True after a non-empty costmap ingest.
  bool goal_ready_{false};              ///< True after SetTerminalGoal.
  bool state_ready_{false};             ///< True after the first successful command.
  State goal_;                          ///< Working goal. Hover evasion may move it.
  State terminal_;                      ///< User goal. Restored when the hover point is clear.
  State robot_;                         ///< Last measured state passed to ComputeCommand.
  Status status_{Status::kYawing};      ///< Mode that gates replanning and the yaw filter.
  std::deque<State> plan_;              ///< Sampled setpoints. The front is the command; the tail gates kGoalSeen.
  std::vector<Eigen::Vector2d> global_path_;  ///< Last geometric path, used as the direction hint.
  ObstacleTracker obstacle_tracker_;             ///< Costmap clusters and external movers.
  HoverMonitor hover_monitor_;                   ///< Slides the working goal while hover is latched.
  PathSearcher path_searcher_;                    ///< Heat-weighted search and path post-process.
  GeometricPath geometric_path_;                 ///< Resampling and unknown-space truncation.
  SafeCorridor safe_corridor_;                   ///< Time-layered polyhedral corridor.
  TrajectoryOptimizer trajectory_optimizer_;     ///< Min-jerk MIQP or penalized quadratic program.
  double yaw_start_x_{0.0};             ///< World x latched when yawing begins.
  double yaw_start_y_{0.0};             ///< World y latched when yawing begins.
  double previous_yaw_{0.0};            ///< Last commanded yaw, radians. The yaw step integrates from here.
  std::chrono::steady_clock::time_point yaw_start_{};  ///< Start of the current yawing interval, for the 10 s timeout.
  double hover_x_{0.0};                 ///< Latched hover x. Not overwritten by evasion.
  double hover_y_{0.0};                 ///< Latched hover y.
  double goal_distance_{1.0e9};             ///< Distance from robot_ to terminal_, meters.
  double goal_yaw_error_{0.0};            ///< Wrapped terminal yaw error, radians.
  double last_replan_time_{0.0};        ///< Clock time of the last replan attempt, seconds.
  double last_command_time_{0.0};       ///< Clock time through which plan samples have been consumed, seconds.
  double estimated_computation_time_{0.05};          ///< Filtered solve time, seconds. Used only after adapt_commit_length_ is set.
  int replan_count_{0};                 ///< Successful replans. The first one is not stored in the time filter.
  std::vector<double> computation_times_;      ///< Solve times collected before adaptation starts. Averaged at 10 samples.
  int failure_count_{0};                ///< Consecutive replans that did not yield a limit-satisfying trajectory.
  int committed_sample_count_{0};                      ///< Samples kept from the previous plan. Non-adapt mode uses default_k.
  bool adapt_commit_length_{false};                 ///< True after ten successful solve times have been averaged.
  double last_factor_{1.5};             ///< Center of the next factor window. Shifts up after a full miss.
};

}  // namespace sando_controller
}  // namespace controller
}  // namespace control
}  // namespace autonomy
