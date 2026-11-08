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

#include "file_replay_source.hpp"

#include <algorithm>
#include <array>
#include <cctype>
#include <cmath>
#include <cstdlib>
#include <fstream>
#include <sstream>
#include <utility>

namespace atlas {
namespace app {
namespace {

bool IsEmptyOrComment(const std::string& line) {
  const std::size_t first_character = line.find_first_not_of(" \t\r\n");
  return first_character == std::string::npos || line[first_character] == '#';
}

template <std::size_t ValueCount>
bool ParseFixedDoubles(const std::string& line,
                       std::array<double, ValueCount>* values) {
  if (values == nullptr) {
    return false;
  }
  const char* cursor = line.c_str();
  const char* const line_end = cursor + line.size();
  for (double& value : *values) {
    while (cursor < line_end &&
           std::isspace(static_cast<unsigned char>(*cursor))) {
      ++cursor;
    }
    if (cursor == line_end) {
      return false;
    }
    char* parsed_end = nullptr;
    value = std::strtod(cursor, &parsed_end);
    if (parsed_end == cursor || parsed_end > line_end ||
        !std::isfinite(value)) {
      return false;
    }
    cursor = parsed_end;
  }
  while (cursor < line_end &&
         std::isspace(static_cast<unsigned char>(*cursor))) {
    ++cursor;
  }
  return cursor == line_end;
}

std::string JoinPath(const std::string& directory,
                     const std::string& relative_path) {
  if (relative_path.empty() || relative_path.front() == '/') {
    return relative_path;
  }
  if (directory.empty() || directory.back() == '/') {
    return directory + relative_path;
  }
  return directory + "/" + relative_path;
}

bool FileExists(const std::string& path) {
  std::ifstream input(path);
  return static_cast<bool>(input);
}

}  // namespace

bool FileReplaySource::Load(const std::string& directory,
                            double point_time_scale_to_seconds) {
  if (directory.empty() || !std::isfinite(point_time_scale_to_seconds) ||
      point_time_scale_to_seconds <= 0.0) {
    return false;
  }
  std::ifstream imu_input(JoinPath(directory, "imu.txt"));
  std::ifstream scans_input(JoinPath(directory, "scans.txt"));
  if (!imu_input || !scans_input) {
    return false;
  }

  std::vector<sensor::ImuData> imu_samples;
  std::string line;
  while (std::getline(imu_input, line)) {
    if (IsEmptyOrComment(line)) {
      continue;
    }
    std::array<double, 7> values;
    if (!ParseFixedDoubles(line, &values)) {
      return false;
    }
    sensor::ImuData sample;
    sample.time = values[0];
    sample.frame_id = "imu";
    sample.angular_velocity = Eigen::Vector3d(values[1], values[2], values[3]);
    sample.linear_acceleration =
        Eigen::Vector3d(values[4], values[5], values[6]);
    imu_samples.push_back(sample);
  }

  std::vector<ScanFile> scans;
  while (std::getline(scans_input, line)) {
    if (IsEmptyOrComment(line)) {
      continue;
    }
    std::istringstream stream(line);
    double time = 0.0;
    std::string relative_path;
    if (!(stream >> time >> relative_path) || !std::isfinite(time) ||
        relative_path.empty()) {
      return false;
    }
    std::string extra;
    if (stream >> extra || !FileExists(JoinPath(directory, relative_path))) {
      return false;
    }
    scans.push_back(ScanFile{time, std::move(relative_path)});
  }

  std::sort(imu_samples.begin(), imu_samples.end(),
            [](const sensor::ImuData& left, const sensor::ImuData& right) {
              return left.time < right.time;
            });
  std::sort(scans.begin(), scans.end(),
            [](const ScanFile& left, const ScanFile& right) {
              return left.time < right.time;
            });

  directory_ = directory;
  point_time_scale_to_seconds_ = point_time_scale_to_seconds;
  imu_ = std::move(imu_samples);
  scans_ = std::move(scans);
  return true;
}

bool FileReplaySource::LoadScan(std::size_t index,
                                sensor::PointCloudXYZI* cloud,
                                std::string* error) const {
  if (cloud == nullptr || index >= scans_.size()) {
    if (error != nullptr) {
      *error = "scan index out of range";
    }
    return false;
  }
  const std::string path = JoinPath(directory_, scans_[index].path);
  std::ifstream input(path);
  if (!input) {
    if (error != nullptr) {
      *error = "cannot open " + path;
    }
    return false;
  }

  sensor::PointCloudXYZI loaded_cloud;
  std::string line;
  std::size_t line_number = 0;
  bool curvature_is_sorted = true;
  while (std::getline(input, line)) {
    ++line_number;
    if (IsEmptyOrComment(line)) {
      continue;
    }
    std::array<double, 5> values;
    if (!ParseFixedDoubles(line, &values) || values[4] < 0.0) {
      if (error != nullptr) {
        *error =
            path + ":" + std::to_string(line_number) + ": malformed scan row";
      }
      return false;
    }
    sensor::PointType point;
    point.x = static_cast<float>(values[0]);
    point.y = static_cast<float>(values[1]);
    point.z = static_cast<float>(values[2]);
    point.intensity = static_cast<float>(values[3]);
    point.curvature =
        static_cast<float>(values[4] * point_time_scale_to_seconds_);
    if (!loaded_cloud.empty() &&
        point.curvature < loaded_cloud.points.back().curvature) {
      curvature_is_sorted = false;
    }
    loaded_cloud.push_back(point);
  }

  if (!curvature_is_sorted) {
    std::sort(loaded_cloud.points.begin(), loaded_cloud.points.end(),
              [](const sensor::PointType& left,
                 const sensor::PointType& right) {
                return left.curvature < right.curvature;
              });
  }
  *cloud = std::move(loaded_cloud);
  return true;
}

}  // namespace app
}  // namespace atlas
