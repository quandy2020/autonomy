/*
 * Copyright 2026 The Openbot Authors
 *
 * STOMP parameters (MoveIt stomp planning yaml lite).
 */

#pragma once

#include <string>

namespace autonomy {
namespace manipulation {
namespace planner {

/** @brief STOMP optimizer knobs. */
struct StompParams {
  int num_iterations = 10;
  int num_iterations_after_valid = 2;
  int num_timesteps = 20;
  int num_rollouts = 10;
  double noise_stddev = 0.05;
  double collision_penalty = 10.0;
  double control_cost_weight = 0.1;
  double exponentiated_cost_sensitivity = 10.0;
  double planning_time_limit = 0.0;  // s; 0 = iterations only
  /** Random mid-waypoint restarts after a failed optimize. */
  bool enable_failure_recovery = true;
  int max_recovery_attempts = 2;
};

inline StompParams DefaultStompParams() { return StompParams{}; }

bool LoadStompParamsFile(const std::string& path, StompParams* params,
                         std::string* error = nullptr);

bool LoadStompParamsFromShare(StompParams* params, std::string* error = nullptr);

}  // namespace planner
}  // namespace manipulation
}  // namespace autonomy
