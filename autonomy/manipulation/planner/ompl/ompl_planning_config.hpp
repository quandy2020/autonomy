/*
 * Copyright 2026 The Openbot Authors
 *
 * Load OmplPlannerConfig table from text (MoveIt ompl_planning.yaml lite).
 */

#pragma once

#include <string>
#include <vector>

namespace autonomy {
namespace manipulation {
namespace planning {

/** @brief One named OMPL planner configuration (MoveIt PlannerConfiguration). */
struct OmplPlannerConfig {
  std::string name;          // e.g. "arm[RRTConnect]"
  std::string planner_id;    // RRTConnect / RRTstar / …
  double planning_time = 1.0;
  int max_attempts = 3;
  /** @brief GoalState / ConstrainedGoalRegion threshold (joint rad). */
  double goal_joint_tolerance = 1e-3;
  /** @brief Longest valid segment fraction for SI (MoveIt longest_valid_segment). */
  double longest_valid_segment_fraction = 0.01;
};

/**
 * @brief Parse `ompl_planning.conf` lines: name planner_id time attempts.
 * @return true if at least one config parsed (empty file → false).
 */
bool LoadOmplPlannerConfigsFile(const std::string& path,
                                std::vector<OmplPlannerConfig>* configs,
                                std::string* error = nullptr);

/** @brief Built-in defaults matching conf/ompl_planning.conf. */
std::vector<OmplPlannerConfig> DefaultOmplPlannerConfigs();

}  // namespace planning
}  // namespace manipulation
}  // namespace autonomy
