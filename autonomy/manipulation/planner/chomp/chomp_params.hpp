/*
 * Copyright 2026 The Openbot Authors
 *
 * CHOMP parameters (MoveIt chomp_planning.yaml lite).
 */

#pragma once

#include <string>
#include <vector>

namespace autonomy {
namespace manipulation {
namespace planner {

/** @brief CHOMP optimizer knobs aligned with MoveIt defaults. */
struct ChompParams {
  int max_iterations = 50;
  int max_iterations_after_collision_free = 5;
  int num_timesteps = 30;
  double learning_rate = 0.1;
  double smoothness_cost_weight = 0.1;
  double smoothness_cost_velocity = 0.0;
  double smoothness_cost_acceleration = 1.0;
  double smoothness_cost_jerk = 0.0;
  double obstacle_cost_weight = 1.0;
  /** Clearance ε for MoveIt getPotential (m). */
  double min_clearance = 0.05;
  double ridge_factor = 1e-4;
  double joint_update_limit = 0.1;
  bool filter_update = true;
  double voxel_resolution = 0.05;
  double voxel_padding = 0.25;
  double voxel_margin = 0.4;
  /**
   * Seed trajectory: "linear" | "cubic" | "quintic"
   * (MoveIt trajectory_initialization_method lite).
   */
  std::string trajectory_initialization_method = "linear";
  /** Random mid-waypoint restarts after a failed optimize. */
  bool enable_failure_recovery = true;
  int max_recovery_attempts = 2;
  /** Wall-clock budget (s); 0 = only max_iterations. */
  double planning_time_limit = 0.0;
};

/** @brief Built-in MoveIt-like defaults. */
inline ChompParams DefaultChompParams() { return ChompParams{}; }

/**
 * @brief Parse `chomp_planning.conf` key=value lines.
 * @return true if file opened (unknown keys ignored).
 */
bool LoadChompParamsFile(const std::string& path, ChompParams* params,
                         std::string* error = nullptr);

/**
 * @brief Resolve share conf then load; on failure keep @p params unchanged.
 */
bool LoadChompParamsFromShare(ChompParams* params, std::string* error = nullptr);

}  // namespace planner
}  // namespace manipulation
}  // namespace autonomy
