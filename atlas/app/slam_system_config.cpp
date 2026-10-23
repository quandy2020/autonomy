/*
 * Copyright 2026 The Openbot Authors
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

#include "slam_system_config.hpp"

#include <cmath>
#include <exception>
#include <fstream>
#include <string>
#include <utility>
#include <vector>

namespace atlas {
namespace app {
namespace {

std::string Trim(std::string text) {
  const auto begin = text.find_first_not_of(" \t\r\n");
  if (begin == std::string::npos) {
    return {};
  }
  const auto end = text.find_last_not_of(" \t\r\n");
  return text.substr(begin, end - begin + 1);
}

std::string StripComment(const std::string& text) {
  bool quoted = false;
  for (std::size_t i = 0; i < text.size(); ++i) {
    if (text[i] == '"') {
      quoted = !quoted;
    } else if (text[i] == '#' && !quoted) {
      return text.substr(0, i);
    }
  }
  return text;
}

std::string Unquote(std::string text) {
  if (text.size() >= 2 && text.front() == '"' && text.back() == '"') {
    return text.substr(1, text.size() - 2);
  }
  return text;
}

struct ParsedLine {
  int indent = 0;
  bool list_item = false;
  bool has_value = false;
  std::string key;
  std::string value;
};

bool ParseLine(const std::string& raw, ParsedLine* line) {
  std::string text = raw;
  for (char& character : text) {
    if (character == '\t') {
      character = ' ';
    }
  }
  text = Trim(StripComment(text));
  if (text.empty()) {
    return false;
  }
  line->indent = 0;
  while (line->indent < static_cast<int>(raw.size()) &&
         (raw[line->indent] == ' ' || raw[line->indent] == '\t')) {
    ++line->indent;
  }
  line->list_item = false;
  if (!text.empty() && text[0] == '-') {
    line->list_item = true;
    text = Trim(text.substr(1));
  }
  const auto colon = text.find(':');
  if (colon == std::string::npos) {
    return false;
  }
  line->key = Trim(text.substr(0, colon));
  const std::string remainder = Trim(text.substr(colon + 1));
  line->has_value = !remainder.empty();
  line->value = Unquote(remainder);
  return !line->key.empty();
}

bool ParseInt(const std::string& text, int* value) {
  try {
    std::size_t used = 0;
    const int parsed = std::stoi(text, &used);
    if (used != text.size()) {
      return false;
    }
    *value = parsed;
    return true;
  } catch (const std::exception&) {
    return false;
  }
}

bool ParseSize(const std::string& text, std::size_t* value) {
  try {
    std::size_t used = 0;
    const long long parsed = std::stoll(text, &used);
    if (used != text.size() || parsed < 0) {
      return false;
    }
    *value = static_cast<std::size_t>(parsed);
    return true;
  } catch (const std::exception&) {
    return false;
  }
}

bool ParseDouble(const std::string& text, double* value) {
  try {
    std::size_t used = 0;
    const double parsed = std::stod(text, &used);
    if (used != text.size() || !std::isfinite(parsed)) {
      return false;
    }
    *value = parsed;
    return true;
  } catch (const std::exception&) {
    return false;
  }
}

bool ParseBool(const std::string& text, bool* value) {
  if (text == "true" || text == "1" || text == "yes" || text == "on") {
    *value = true;
    return true;
  }
  if (text == "false" || text == "0" || text == "no" || text == "off") {
    *value = false;
    return true;
  }
  return false;
}

bool AssignScalar(const std::string& path, const std::string& value,
                  SlamSystemConfig* config) {
  if (path == "pipeline") {
    config->pipeline = value;
    return true;
  }
  if (path == "trajectory_path") {
    config->trajectory_path = value;
    return true;
  }
  if (path == "min_imu_samples" || path == "initialization/min_imu_samples") {
    return ParseInt(value, &config->min_imu_samples);
  }
  if (path == "gravity_magnitude" ||
      path == "initialization/gravity_magnitude") {
    return ParseDouble(value, &config->gravity_magnitude);
  }
  if (path == "max_accel_variance" ||
      path == "initialization/max_accel_variance") {
    return ParseDouble(value, &config->max_accel_variance);
  }
  if (path == "window_size" || path == "local_mapping/window_size") {
    return ParseSize(value, &config->window_size);
  }
  if (path == "voxel/win_size") {
    return ParseInt(value, &config->voxel_win_size);
  }
  if (path == "voxel/thread_num") {
    return ParseInt(value, &config->voxel_thread_num);
  }
  if (path == "voxel/degrade_bound") {
    return ParseInt(value, &config->voxel_degrade_bound);
  }
  if (path == "voxel/down_size") {
    return ParseDouble(value, &config->voxel_down_size);
  }
  if (path == "voxel/voxel_size") {
    return ParseDouble(value, &config->voxel_size);
  }
  if (path == "voxel/min_eigen_value") {
    return ParseDouble(value, &config->voxel_min_eigen_value);
  }
  if (path == "voxel/imu_coef") {
    return ParseDouble(value, &config->voxel_imu_coef);
  }
  if (path == "voxel/enable_loop") {
    return ParseBool(value, &config->voxel_enable_loop);
  }
  if (path == "voxel/enable_gba") {
    return ParseBool(value, &config->voxel_enable_gba);
  }
  if (path == "voxel/enable_local_ba") {
    return ParseBool(value, &config->voxel_enable_local_ba);
  }
  if (path == "voxel/enable_gravity_ba") {
    return ParseBool(value, &config->voxel_enable_gravity_ba);
  }
  if (path == "voxel/enable_triangle") {
    return ParseBool(value, &config->voxel_enable_triangle);
  }
  if (path == "voxel/icp_eigval") {
    return ParseDouble(value, &config->voxel_icp_eigval);
  }
  if (path == "voxel/jud_default") {
    return ParseDouble(value, &config->voxel_jud_default);
  }
  if (path == "voxel/ratio_drift") {
    return ParseDouble(value, &config->voxel_ratio_drift);
  }
  if (path == "voxel/curr_halt") {
    return ParseInt(value, &config->voxel_curr_halt);
  }
  if (path == "voxel/prev_halt") {
    return ParseInt(value, &config->voxel_prev_halt);
  }
  if (path == "voxel/gba_local_win") {
    return ParseInt(value, &config->voxel_gba_local_win);
  }
  if (path == "voxel/gba_merge_size") {
    return ParseInt(value, &config->voxel_gba_merge_size);
  }
  if (path == "voxel/save_path") {
    config->voxel_save_path = value;
    return true;
  }
  if (path == "voxel/bagname") {
    config->voxel_bagname = value;
    return true;
  }
  if (path == "voxel/previous_map") {
    config->voxel_previous_map = value;
    return true;
  }
  if (path == "components/name") {
    if (!config->components.empty() && config->components.back().type.empty()) {
      return false;
    }
    config->components.push_back(ComponentSpec{value, {}});
    return !value.empty();
  }
  if (path == "components/type") {
    if (config->components.empty() || !config->components.back().type.empty()) {
      return false;
    }
    config->components.back().type = value;
    return !value.empty();
  }
  return false;
}

}  // namespace

SlamSystemConfig DefaultSlamSystemConfig() { return SlamSystemConfig{}; }

bool ValidateSlamSystemConfig(const SlamSystemConfig& config) {
  if (config.pipeline != "lidar" && config.pipeline != "visual" &&
      config.pipeline != "voxel_slam") {
    return false;
  }
  if (config.min_imu_samples < 1 || !(config.gravity_magnitude > 0.0) ||
      config.max_accel_variance < 0.0 || config.window_size < 1) {
    return false;
  }
  if (config.voxel_win_size < 1 || config.voxel_thread_num < 1 ||
      config.voxel_degrade_bound < 1 || !(config.voxel_down_size > 0.0) ||
      !(config.voxel_size > 0.0) || !(config.voxel_min_eigen_value > 0.0) ||
      !(config.voxel_imu_coef > 0.0) || !(config.voxel_icp_eigval >= 0.0) ||
      !(config.voxel_jud_default >= 0.0) ||
      !(config.voxel_ratio_drift >= 0.0) || config.voxel_curr_halt < 1 ||
      config.voxel_prev_halt < 1 || config.voxel_gba_local_win < 2 ||
      config.voxel_gba_merge_size < 1) {
    return false;
  }
  for (std::size_t i = 0; i < config.components.size(); ++i) {
    if (config.components[i].name.empty() ||
        config.components[i].type.empty()) {
      return false;
    }
    for (std::size_t j = 0; j < i; ++j) {
      if (config.components[j].name == config.components[i].name) {
        return false;
      }
    }
  }
  return true;
}

bool LoadSlamSystemConfig(const std::string& path, SlamSystemConfig* config) {
  if (config == nullptr || path.empty()) {
    return false;
  }
  std::ifstream input(path);
  if (!input) {
    return false;
  }
  SlamSystemConfig loaded = DefaultSlamSystemConfig();
  struct Frame {
    int indent = 0;
    std::string path;
  };
  std::vector<Frame> stack;
  bool saw_components = false;
  std::string raw;
  while (std::getline(input, raw)) {
    ParsedLine line;
    if (!ParseLine(raw, &line)) {
      if (Trim(StripComment(raw)).empty()) {
        continue;
      }
      return false;
    }
    while (!stack.empty() && stack.back().indent >= line.indent) {
      stack.pop_back();
    }
    const std::string parent =
        stack.empty() ? std::string() : stack.back().path;
    const std::string full =
        parent.empty() ? line.key : parent + "/" + line.key;
    if (line.list_item && parent != "components") {
      return false;
    }
    if (!line.has_value) {
      if (full == "components") {
        if (saw_components) {
          return false;
        }
        saw_components = true;
        loaded.components.clear();
      } else if (full != "initialization" && full != "local_mapping" &&
                 full != "voxel") {
        return false;
      }
      stack.push_back(Frame{line.indent, full});
      continue;
    }
    if (!AssignScalar(full, line.value, &loaded)) {
      return false;
    }
  }
  if (!ValidateSlamSystemConfig(loaded)) {
    return false;
  }
  *config = std::move(loaded);
  return true;
}

bool SaveSlamSystemConfig(const SlamSystemConfig& config,
                          const std::string& path) {
  if (path.empty()) {
    return false;
  }
  std::ofstream output(path);
  if (!output) {
    return false;
  }
  output.precision(17);
  output << "pipeline: " << config.pipeline << "\n";
  output << "trajectory_path: \"" << config.trajectory_path << "\"\n";
  output << "initialization:\n";
  output << "  min_imu_samples: " << config.min_imu_samples << "\n";
  output << "  gravity_magnitude: " << config.gravity_magnitude << "\n";
  output << "  max_accel_variance: " << config.max_accel_variance << "\n";
  output << "local_mapping:\n";
  output << "  window_size: " << config.window_size << "\n";
  output << "voxel:\n";
  output << "  win_size: " << config.voxel_win_size << "\n";
  output << "  thread_num: " << config.voxel_thread_num << "\n";
  output << "  degrade_bound: " << config.voxel_degrade_bound << "\n";
  output << "  down_size: " << config.voxel_down_size << "\n";
  output << "  voxel_size: " << config.voxel_size << "\n";
  output << "  min_eigen_value: " << config.voxel_min_eigen_value << "\n";
  output << "  imu_coef: " << config.voxel_imu_coef << "\n";
  output << "  enable_loop: " << (config.voxel_enable_loop ? "true" : "false")
         << "\n";
  output << "  enable_gba: " << (config.voxel_enable_gba ? "true" : "false")
         << "\n";
  output << "  enable_local_ba: "
         << (config.voxel_enable_local_ba ? "true" : "false") << "\n";
  output << "  enable_gravity_ba: "
         << (config.voxel_enable_gravity_ba ? "true" : "false") << "\n";
  output << "  enable_triangle: "
         << (config.voxel_enable_triangle ? "true" : "false") << "\n";
  output << "  icp_eigval: " << config.voxel_icp_eigval << "\n";
  output << "  jud_default: " << config.voxel_jud_default << "\n";
  output << "  ratio_drift: " << config.voxel_ratio_drift << "\n";
  output << "  curr_halt: " << config.voxel_curr_halt << "\n";
  output << "  prev_halt: " << config.voxel_prev_halt << "\n";
  output << "  gba_local_win: " << config.voxel_gba_local_win << "\n";
  output << "  gba_merge_size: " << config.voxel_gba_merge_size << "\n";
  output << "  save_path: \"" << config.voxel_save_path << "\"\n";
  output << "  bagname: \"" << config.voxel_bagname << "\"\n";
  output << "  previous_map: \"" << config.voxel_previous_map << "\"\n";
  output << "components:\n";
  for (const ComponentSpec& component : config.components) {
    output << "  - name: " << component.name << "\n";
    output << "    type: " << component.type << "\n";
  }
  return static_cast<bool>(output);
}

}  // namespace app
}  // namespace atlas
