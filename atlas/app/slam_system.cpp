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

#include "slam_system.hpp"

#include <algorithm>
#include <cmath>
#include <fstream>
#include <sstream>
#include <utility>

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

bool ParseDoubles(const std::string& line, std::vector<double>* values) {
  std::istringstream stream(line);
  values->clear();
  double value = 0.0;
  while (stream >> value) {
    if (!std::isfinite(value)) {
      return false;
    }
    values->push_back(value);
  }
  return stream.eof() && !values->empty();
}

std::string JoinPath(const std::string& directory,
                     const std::string& relative) {
  if (relative.empty() || relative.front() == '/') {
    return relative;
  }
  if (directory.empty() || directory.back() == '/') {
    return directory + relative;
  }
  return directory + "/" + relative;
}

bool FileExists(const std::string& path) {
  std::ifstream input(path);
  return static_cast<bool>(input);
}

}  // namespace

bool FileReplaySource::Load(const std::string& directory) {
  if (directory.empty()) {
    return false;
  }
  const std::string imu_path = JoinPath(directory, "imu.txt");
  const std::string scans_path = JoinPath(directory, "scans.txt");
  std::ifstream imu_input(imu_path);
  std::ifstream scans_input(scans_path);
  if (!imu_input || !scans_input) {
    return false;
  }

  std::vector<sensor::ImuData> imu;
  std::string line;
  while (std::getline(imu_input, line)) {
    line = Trim(line);
    if (line.empty() || line[0] == '#') {
      continue;
    }
    std::vector<double> values;
    if (!ParseDoubles(line, &values) || values.size() != 7) {
      return false;
    }
    sensor::ImuData sample;
    sample.time = values[0];
    sample.frame_id = "imu";
    sample.angular_velocity = Eigen::Vector3d(values[1], values[2], values[3]);
    sample.linear_acceleration =
        Eigen::Vector3d(values[4], values[5], values[6]);
    imu.push_back(sample);
  }

  std::vector<ScanRef> scans;
  while (std::getline(scans_input, line)) {
    line = Trim(line);
    if (line.empty() || line[0] == '#') {
      continue;
    }
    std::istringstream stream(line);
    double time = 0.0;
    std::string relative;
    if (!(stream >> time >> relative) || !std::isfinite(time) ||
        relative.empty()) {
      return false;
    }
    std::string extra;
    if (stream >> extra) {
      return false;
    }
    if (!FileExists(JoinPath(directory, relative))) {
      return false;
    }
    scans.push_back(ScanRef{time, relative});
  }

  std::sort(imu.begin(), imu.end(),
            [](const sensor::ImuData& lhs, const sensor::ImuData& rhs) {
              return lhs.time < rhs.time;
            });
  std::sort(scans.begin(), scans.end(),
            [](const ScanRef& lhs, const ScanRef& rhs) {
              return lhs.time < rhs.time;
            });
  directory_ = directory;
  imu_ = std::move(imu);
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
  sensor::PointCloudXYZI loaded;
  std::string line;
  std::size_t line_number = 0;
  while (std::getline(input, line)) {
    ++line_number;
    line = Trim(line);
    if (line.empty() || line[0] == '#') {
      continue;
    }
    std::vector<double> values;
    if (!ParseDoubles(line, &values) || values.size() != 5 || values[4] < 0.0) {
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
    point.curvature = static_cast<float>(values[4]);
    loaded.push_back(point);
  }
  std::sort(loaded.points.begin(), loaded.points.end(),
            [](const sensor::PointType& lhs, const sensor::PointType& rhs) {
              return lhs.curvature < rhs.curvature;
            });
  *cloud = std::move(loaded);
  return true;
}

SlamSystem::SlamSystem(SlamSystemConfig config)
    : config_(std::move(config)),
      initializer_(frontend::lidar::Initializer::Config{
          config_.min_imu_samples, 3, config_.gravity_magnitude, 0.5,
          config_.max_accel_variance}),
      local_mapping_(
          frontend::lidar::LocalMapping::Config{config_.window_size}) {}

bool SlamSystem::Start() {
  if (!ValidateSlamSystemConfig(config_)) {
    return false;
  }
  started_ = true;
  return true;
}

void SlamSystem::Stop() { started_ = false; }

void SlamSystem::AddImu(const sensor::ImuData& imu) {
  if (!started_ || !std::isfinite(imu.time)) {
    return;
  }
  imu_.push_back(imu);
}

bool SlamSystem::AddScan(double time) {
  if (!started_ || !std::isfinite(time) || config_.pipeline != "lidar") {
    return false;
  }
  if (!initialized_) {
    frontend::lidar::State state;
    if (!initializer_.TryInitialize(imu_, &state)) {
      return true;
    }
    initialized_ = true;
    state_ = state;
    local_mapping_.Add(state_);
    trajectory_.push_back(state_);
    imu_.erase(std::remove_if(imu_.begin(), imu_.end(),
                              [&state](const sensor::ImuData& sample) {
                                return sample.time <= state.time;
                              }),
               imu_.end());
    return true;
  }

  std::vector<sensor::ImuData> pending;
  for (const sensor::ImuData& sample : imu_) {
    if (sample.time > state_.time && sample.time <= time) {
      pending.push_back(sample);
    }
  }
  const frontend::lidar::Odometry::PredictResult predicted =
      odometry_.Predict(state_, pending);
  if (predicted.propagated) {
    state_ = predicted.state;
  }
  if (state_.time < time) {
    state_.time = time;
  }
  local_mapping_.Add(state_);
  trajectory_.push_back(state_);
  imu_.erase(std::remove_if(imu_.begin(), imu_.end(),
                            [time](const sensor::ImuData& sample) {
                              return sample.time <= time;
                            }),
             imu_.end());
  return true;
}

bool SlamSystem::WriteTrajectory(const std::string& path) const {
  if (path.empty()) {
    return false;
  }
  std::ofstream output(path);
  if (!output) {
    return false;
  }
  output.setf(std::ios::fixed);
  output.precision(9);
  for (const frontend::lidar::State& state : trajectory_) {
    const Eigen::Vector3d& translation = state.pose.translation();
    const Eigen::Quaterniond& rotation = state.pose.rotation();
    output << state.time << " " << translation.x() << " " << translation.y()
           << " " << translation.z() << " " << rotation.x() << " "
           << rotation.y() << " " << rotation.z() << " " << rotation.w()
           << "\n";
  }
  return static_cast<bool>(output);
}

int ReplayDataset(SlamSystem* system, const FileReplaySource& source) {
  if (system == nullptr || !system->started()) {
    return -1;
  }
  std::size_t imu_index = 0;
  int accepted = 0;
  for (const ScanRef& scan : source.scans()) {
    while (imu_index < source.imu().size() &&
           source.imu()[imu_index].time <= scan.time) {
      system->AddImu(source.imu()[imu_index]);
      ++imu_index;
    }
    if (!system->AddScan(scan.time)) {
      return -1;
    }
    ++accepted;
  }
  while (imu_index < source.imu().size()) {
    system->AddImu(source.imu()[imu_index]);
    ++imu_index;
  }
  return accepted;
}

}  // namespace app
}  // namespace atlas
