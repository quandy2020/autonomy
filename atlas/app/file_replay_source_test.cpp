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

#include <gtest/gtest.h>

#include <chrono>
#include <filesystem>
#include <fstream>
#include <string>

#include "voxel_slam_system.hpp"

namespace atlas {
namespace app {
namespace {

void WriteSyntheticDataset(const std::filesystem::path& directory) {
  std::ofstream imu_file(directory / "imu.txt");
  for (int sample_index = 0; sample_index < 41; ++sample_index) {
    const double time = sample_index * 0.01;
    imu_file << time << " 0 0 0 0 0 9.81\n";
  }
  imu_file.close();

  std::ofstream scan_index_file(directory / "scans.txt");
  scan_index_file << "0.20 scan_0.txt\n";
  scan_index_file << "0.30 scan_1.txt\n";
  scan_index_file.close();

  for (int scan_index = 0; scan_index < 2; ++scan_index) {
    std::ofstream scan_file(directory /
                            ("scan_" + std::to_string(scan_index) + ".txt"));
    scan_file << "0 0 0 1 0\n";
    scan_file << "1 0 0 1 0.01\n";
    scan_file << "0 1 0 1 0.02\n";
    scan_file << "0 0 1 1 0.03\n";
  }
}

TEST(FileReplaySource, RejectsDatasetThatProducesNoTrajectory) {
  const auto unique_suffix =
      std::chrono::steady_clock::now().time_since_epoch().count();
  const std::filesystem::path temporary_directory =
      std::filesystem::temp_directory_path() /
      ("atlas_file_replay_test_" + std::to_string(unique_suffix));
  std::filesystem::create_directories(temporary_directory);
  WriteSyntheticDataset(temporary_directory);

  FileReplaySource source;
  ASSERT_TRUE(source.Load(temporary_directory.string()));
  ASSERT_EQ(source.scan_count(), 2u);
  ASSERT_EQ(source.imu().size(), 41u);

  SlamSystemConfig config = DefaultSlamSystemConfig();
  config.voxel_slam.global_mapping.enabled = false;
  VoxelSlamSystem system(config);
  ASSERT_TRUE(system.Start());
  EXPECT_EQ(ReplayVoxelDataset(&system, source), -1);
  EXPECT_FALSE(system.initialized());
  ASSERT_TRUE(system.Stop());

  std::error_code error;
  std::filesystem::remove_all(temporary_directory, error);
}

TEST(FileReplaySource, ConvertsMillisecondPointTimesToSeconds) {
  const auto unique_suffix =
      std::chrono::steady_clock::now().time_since_epoch().count();
  const std::filesystem::path temporary_directory =
      std::filesystem::temp_directory_path() /
      ("atlas_file_replay_time_test_" + std::to_string(unique_suffix));
  std::filesystem::create_directories(temporary_directory);
  WriteSyntheticDataset(temporary_directory);
  {
    std::ofstream scan_file(temporary_directory / "scan_0.txt");
    scan_file << "1 0 0 1 100\n";
  }

  FileReplaySource source;
  ASSERT_TRUE(source.Load(temporary_directory.string(), 0.001));
  sensor::PointCloudXYZI cloud;
  ASSERT_TRUE(source.LoadScan(0, &cloud, nullptr));
  ASSERT_EQ(cloud.size(), 1u);
  EXPECT_FLOAT_EQ(cloud.points.front().curvature, 0.1f);

  std::error_code error;
  std::filesystem::remove_all(temporary_directory, error);
}

}  // namespace
}  // namespace app
}  // namespace atlas
