/*
 * Copyright 2026 The Openbot Authors
 */

#include "autonomy/manipulation/planner/stomp/stomp_params.hpp"

#include <fstream>

#include "autonomy/common/conf_loader.hpp"

namespace autonomy {
namespace manipulation {
namespace planner {
namespace {

bool ApplyKey(const std::string& key, const std::string& val, StompParams* p) {
  if (!p) {
    return false;
  }
  try {
    if (key == "num_iterations") {
      p->num_iterations = std::stoi(val);
    } else if (key == "num_iterations_after_valid") {
      p->num_iterations_after_valid = std::stoi(val);
    } else if (key == "num_timesteps") {
      p->num_timesteps = std::stoi(val);
    } else if (key == "num_rollouts") {
      p->num_rollouts = std::stoi(val);
    } else if (key == "noise_stddev") {
      p->noise_stddev = std::stod(val);
    } else if (key == "collision_penalty") {
      p->collision_penalty = std::stod(val);
    } else if (key == "control_cost_weight") {
      p->control_cost_weight = std::stod(val);
    } else if (key == "exponentiated_cost_sensitivity") {
      p->exponentiated_cost_sensitivity = std::stod(val);
    } else if (key == "planning_time_limit") {
      p->planning_time_limit = std::stod(val);
    } else if (key == "enable_failure_recovery") {
      p->enable_failure_recovery =
          (val == "true" || val == "1" || val == "True");
    } else if (key == "max_recovery_attempts") {
      p->max_recovery_attempts = std::stoi(val);
    } else {
      return false;
    }
  } catch (...) {
    return false;
  }
  return true;
}

}  // namespace

bool LoadStompParamsFile(const std::string& path, StompParams* params,
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

bool LoadStompParamsFromShare(StompParams* params, std::string* error) {
  if (!params) {
    return false;
  }
  std::string path;
  if (!common::ResolveModuleConfPath("manipulation", "stomp_planning.conf",
                                     &path)) {
    if (error) {
      *error = "stomp_planning.conf not found in share";
    }
    return false;
  }
  return LoadStompParamsFile(path, params, error);
}

}  // namespace planner
}  // namespace manipulation
}  // namespace autonomy
