/*
 * Copyright 2026 The Openbot Authors
 */

#include "autonomy/manipulation/planner/chomp/chomp_params.hpp"

#include <fstream>
#include <sstream>

#include "autonomy/common/conf_loader.hpp"

namespace autonomy {
namespace manipulation {
namespace planning {
namespace {

bool ApplyKey(const std::string& key, const std::string& val,
              ChompParams* p) {
  if (!p) {
    return false;
  }
  try {
    if (key == "max_iterations") {
      p->max_iterations = std::stoi(val);
    } else if (key == "max_iterations_after_collision_free") {
      p->max_iterations_after_collision_free = std::stoi(val);
    } else if (key == "num_timesteps") {
      p->num_timesteps = std::stoi(val);
    } else if (key == "learning_rate") {
      p->learning_rate = std::stod(val);
    } else if (key == "smoothness_cost_weight") {
      p->smoothness_cost_weight = std::stod(val);
    } else if (key == "smoothness_cost_velocity") {
      p->smoothness_cost_velocity = std::stod(val);
    } else if (key == "smoothness_cost_acceleration") {
      p->smoothness_cost_acceleration = std::stod(val);
    } else if (key == "smoothness_cost_jerk") {
      p->smoothness_cost_jerk = std::stod(val);
    } else if (key == "obstacle_cost_weight") {
      p->obstacle_cost_weight = std::stod(val);
    } else if (key == "min_clearance") {
      p->min_clearance = std::stod(val);
    } else if (key == "ridge_factor") {
      p->ridge_factor = std::stod(val);
    } else if (key == "joint_update_limit") {
      p->joint_update_limit = std::stod(val);
    } else if (key == "filter_update") {
      p->filter_update = (val == "1" || val == "true" || val == "True");
    } else if (key == "voxel_resolution") {
      p->voxel_resolution = std::stod(val);
    } else if (key == "voxel_padding") {
      p->voxel_padding = std::stod(val);
    } else if (key == "voxel_margin") {
      p->voxel_margin = std::stod(val);
    } else if (key == "trajectory_initialization_method") {
      p->trajectory_initialization_method = val;
    } else if (key == "enable_failure_recovery") {
      p->enable_failure_recovery =
          (val == "1" || val == "true" || val == "True");
    } else if (key == "max_recovery_attempts") {
      p->max_recovery_attempts = std::stoi(val);
    } else if (key == "planning_time_limit") {
      p->planning_time_limit = std::stod(val);
    } else {
      return false;
    }
  } catch (...) {
    return false;
  }
  return true;
}

}  // namespace

bool LoadChompParamsFile(const std::string& path, ChompParams* params,
                         std::string* error) {
  if (!params) {
    return false;
  }
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
    const auto eq = line.find('=');
    if (eq == std::string::npos) {
      continue;
    }
    std::string key = line.substr(0, eq);
    std::string val = line.substr(eq + 1);
    while (!key.empty() && (key.back() == ' ' || key.back() == '\t')) {
      key.pop_back();
    }
    while (!val.empty() && (val.front() == ' ' || val.front() == '\t')) {
      val.erase(val.begin());
    }
    ApplyKey(key, val, params);
  }
  return true;
}

bool LoadChompParamsFromShare(ChompParams* params, std::string* error) {
  if (!params) {
    return false;
  }
  std::string path;
  if (!common::ResolveModuleConfPath("manipulation", "chomp_planning.conf",
                                     &path)) {
    if (error) {
      *error = "chomp_planning.conf not found in share";
    }
    return false;
  }
  return LoadChompParamsFile(path, params, error);
}

}  // namespace planning
}  // namespace manipulation
}  // namespace autonomy
