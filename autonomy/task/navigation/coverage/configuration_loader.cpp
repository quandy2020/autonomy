// SPDX-License-Identifier: Apache-2.0
/** @file configuration_loader.cpp
 * @brief Native complete-coverage configuration loader implementation. */
#include "autonomy/task/navigation/coverage/configuration_loader.hpp"

#include <fstream>
#include <nlohmann/json.hpp>

#include "autonomy/task/navigation/coverage/robot_params.hpp"
namespace autonomy::task::coverage {
Configuration LoadConfiguration(const std::string& filename) {
  Configuration config;
  if (filename.empty()) return config;
  std::ifstream file(filename);
  if (!file)
    throw RequestException("Cannot read coverage configuration: " + filename);
  nlohmann::json data;
  file >> data;
#define LOAD_COVERAGE_VALUE(name) config.name = data.value(#name, config.name)
  LOAD_COVERAGE_VALUE(robot_width);
  LOAD_COVERAGE_VALUE(operation_width);
  LOAD_COVERAGE_VALUE(min_turning_radius);
  LOAD_COVERAGE_VALUE(linear_curv_change);
  LOAD_COVERAGE_VALUE(cruise_speed);
  LOAD_COVERAGE_VALUE(turn_speed);
  LOAD_COVERAGE_VALUE(headland_width);
  LOAD_COVERAGE_VALUE(route_type);
  LOAD_COVERAGE_VALUE(path_type);
  LOAD_COVERAGE_VALUE(path_continuity_type);
  LOAD_COVERAGE_VALUE(turn_point_distance);
  LOAD_COVERAGE_VALUE(controller_id);
  LOAD_COVERAGE_VALUE(planner_id);
  LOAD_COVERAGE_VALUE(navigation_timeout);
  LOAD_COVERAGE_VALUE(max_transport_poses);
  LOAD_COVERAGE_VALUE(controller_supports_reverse);
#undef LOAD_COVERAGE_VALUE
  RobotParams validate(config);
  return config;
}
}  // namespace autonomy::task::coverage
