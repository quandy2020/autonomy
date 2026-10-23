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

#include <filesystem>
#include <iostream>
#include <string>

#include "slam_system_builder.hpp"
#include "voxel_slam_system.hpp"

namespace {

struct Arguments {
  std::string config;
  std::string dataset;
  std::string trajectory;
  std::string output;
  bool help = false;
};

void PrintUsage(const char* argv0) {
  std::cerr
      << "usage: " << argv0
      << " --config FILE [--dataset DIR] [--trajectory PATH] [--output DIR]\n";
}

bool ParseArguments(int argc, char** argv, Arguments* arguments) {
  for (int index = 1; index < argc; ++index) {
    const std::string flag = argv[index];
    if (flag == "--help" || flag == "-h") {
      arguments->help = true;
      continue;
    }
    if (index + 1 >= argc) {
      return false;
    }
    const std::string value = argv[++index];
    if (flag == "--config") {
      arguments->config = value;
    } else if (flag == "--dataset") {
      arguments->dataset = value;
    } else if (flag == "--trajectory") {
      arguments->trajectory = value;
    } else if (flag == "--output") {
      arguments->output = value;
    } else {
      return false;
    }
  }
  return true;
}

}  // namespace

int main(int argc, char** argv) {
  Arguments arguments;
  if (!ParseArguments(argc, argv, &arguments) || arguments.help ||
      arguments.config.empty()) {
    PrintUsage(argv[0]);
    return arguments.help ? 0 : 2;
  }

  atlas::app::SlamSystemConfig config;
  if (!atlas::app::LoadSlamSystemConfig(arguments.config, &config)) {
    std::cerr << "failed to load config: " << arguments.config << "\n";
    return 1;
  }
  if (!arguments.trajectory.empty()) {
    config.trajectory_path = arguments.trajectory;
  }
  if (!arguments.output.empty()) {
    std::error_code error;
    std::filesystem::create_directories(arguments.output, error);
    config.trajectory_path =
        (std::filesystem::path(arguments.output) / "trajectory.tum").string();
  }

  if (config.pipeline == "voxel_slam") {
    atlas::app::VoxelSlamSystem system(config);
    if (!system.Start()) {
      std::cerr << "failed to start voxel_slam\n";
      return 1;
    }
    if (!arguments.dataset.empty()) {
      atlas::app::FileReplaySource source;
      if (!source.Load(arguments.dataset)) {
        std::cerr << "failed to load dataset: " << arguments.dataset << "\n";
        return 1;
      }
      if (atlas::app::ReplayVoxelDataset(&system, source) < 0) {
        std::cerr << "voxel replay failed\n";
        return 1;
      }
    }
    std::string finalize_error;
    if (!system.Stop(&finalize_error)) {
      std::cerr << "failed to finalize voxel session: " << finalize_error
                << "\n";
      return 1;
    }
    if (!config.trajectory_path.empty() &&
        !system.WriteTrajectory(config.trajectory_path)) {
      std::cerr << "failed to write trajectory: " << config.trajectory_path
                << "\n";
      return 1;
    }
    return 0;
  }

  const std::unique_ptr<atlas::app::SlamSystem> system =
      atlas::app::SlamSystemBuilder{}.Build(config);
  if (system == nullptr || !system->Start()) {
    std::cerr << "failed to build system\n";
    return 1;
  }

  if (!arguments.dataset.empty()) {
    atlas::app::FileReplaySource source;
    if (!source.Load(arguments.dataset)) {
      std::cerr << "failed to load dataset: " << arguments.dataset << "\n";
      return 1;
    }
    if (atlas::app::ReplayDataset(system.get(), source) < 0) {
      std::cerr << "replay failed\n";
      return 1;
    }
  }

  if (!config.trajectory_path.empty() &&
      !system->WriteTrajectory(config.trajectory_path)) {
    std::cerr << "failed to write trajectory: " << config.trajectory_path
              << "\n";
    return 1;
  }
  system->Stop();
  return 0;
}
