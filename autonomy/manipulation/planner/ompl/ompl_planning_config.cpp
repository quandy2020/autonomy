/*
 * Copyright 2026 The Openbot Authors
 */

#include "autonomy/manipulation/planner/ompl/ompl_planning_config.hpp"

#include <fstream>
#include <sstream>

namespace autonomy {
namespace manipulation {
namespace planner {

std::vector<OmplPlannerConfig> DefaultOmplPlannerConfigs() {
  return {
      {"arm[RRTConnect]", "RRTConnect", 1.0, 3, 1e-3, 0.01},
      {"arm[RRT]", "RRT", 1.0, 3, 1e-3, 0.01},
      {"arm[RRTstar]", "RRTstar", 2.0, 2, 1e-3, 0.01},
      {"arm[KPIECE]", "KPIECE", 1.5, 3, 1e-3, 0.01},
      {"arm[PRM]", "PRM", 2.0, 2, 1e-3, 0.01},
      {"arm[BiTRRT]", "BiTRRT", 1.5, 3, 1e-3, 0.01},
      {"arm[EST]", "EST", 1.5, 3, 1e-3, 0.01},
  };
}

bool LoadOmplPlannerConfigsFile(const std::string& path,
                                std::vector<OmplPlannerConfig>* configs,
                                std::string* error) {
  if (!configs) {
    return false;
  }
  configs->clear();
  std::ifstream in(path);
  if (!in) {
    if (error) {
      *error = "cannot open " + path;
    }
    return false;
  }
  std::string line;
  while (std::getline(in, line)) {
    if (line.empty() || line[0] == '#') {
      continue;
    }
    std::istringstream iss(line);
    OmplPlannerConfig c;
    if (!(iss >> c.name >> c.planner_id >> c.planning_time >> c.max_attempts)) {
      continue;
    }
    // Optional: goal_tol  longest_valid_segment_fraction
    double goal_tol = 0.0;
    double lvs = 0.0;
    if (iss >> goal_tol) {
      c.goal_joint_tolerance = goal_tol;
    }
    if (iss >> lvs) {
      c.longest_valid_segment_fraction = lvs;
    }
    configs->push_back(std::move(c));
  }
  return !configs->empty();
}

}  // namespace planner
}  // namespace manipulation
}  // namespace autonomy
