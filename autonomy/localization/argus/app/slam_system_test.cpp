#include "slam_system.hpp"

#include <gtest/gtest.h>

#include <filesystem>
#include <fstream>

namespace argus {
namespace app {
namespace {

sensor::ImuData Still(double time, const Eigen::Vector3d& acceleration =
                                       Eigen::Vector3d(0.0, 0.0, 9.81)) {
  sensor::ImuData sample;
  sample.time = time;
  sample.linear_acceleration = acceleration;
  return sample;
}

SlamSystemConfig TestConfig() {
  SlamSystemConfig config = DefaultSlamSystemConfig();
  config.min_imu_samples = 2;
  config.window_size = 2;
  config.components.clear();
  return config;
}

TEST(SlamSystem, InitializesThenPredicts) {
  SlamSystem system(TestConfig());
  EXPECT_FALSE(system.AddScan(0.0));
  system.AddImu(Still(0.0));
  ASSERT_TRUE(system.Start());
  system.AddImu(Still(0.0));
  system.AddImu(Still(0.01));
  ASSERT_TRUE(system.AddScan(0.01));
  EXPECT_TRUE(system.initialized());
  EXPECT_NEAR(system.state().gravity.z(), -9.81, 1e-9);
  ASSERT_EQ(system.trajectory().size(), 1u);

  system.AddImu(Still(1.01, Eigen::Vector3d(1.0, 0.0, 9.81)));
  ASSERT_TRUE(system.AddScan(1.01));
  EXPECT_NEAR(system.state().pose.translation().x(), 0.5, 1e-9);
  EXPECT_NEAR(system.state().velocity.x(), 1.0, 1e-9);
  EXPECT_EQ(system.local_mapping().size(), 2u);

  system.AddImu(Still(2.01));
  ASSERT_TRUE(system.AddScan(2.01));
  EXPECT_EQ(system.local_mapping().size(), 2u);
  EXPECT_EQ(system.trajectory().size(), 3u);
  EXPECT_DOUBLE_EQ(system.local_mapping().window().front().time, 1.01);
}

TEST(FileReplaySource, ReplaysDatasetIntoTrajectory) {
  const std::filesystem::path directory =
      std::filesystem::temp_directory_path() / "argus_voxel_slam_replay";
  std::filesystem::remove_all(directory);
  std::filesystem::create_directories(directory / "scans");
  {
    std::ofstream imu(directory / "imu.txt");
    imu << "# still\n";
    imu << "0.000000000 0 0 0 0 0 9.81\n";
    imu << "0.010000000 0 0 0 0 0 9.81\n";
    imu << "1.010000000 0 0 0 1 0 9.81\n";
    std::ofstream scans(directory / "scans.txt");
    scans << "0.010000000 scans/000000.txt\n";
    scans << "1.010000000 scans/000001.txt\n";
    std::ofstream scan0(directory / "scans" / "000000.txt");
    scan0 << "0 0 0 0 0\n";
    std::ofstream scan1(directory / "scans" / "000001.txt");
    scan1 << "1 0 0 0 0\n";
  }

  FileReplaySource source;
  ASSERT_TRUE(source.Load(directory.string()));
  ASSERT_EQ(source.imu().size(), 3u);
  ASSERT_EQ(source.scans().size(), 2u);
  EXPECT_EQ(source.scans()[0].path, "scans/000000.txt");

  SlamSystem system(TestConfig());
  EXPECT_EQ(ReplayDataset(&system, source), -1);
  ASSERT_TRUE(system.Start());
  EXPECT_EQ(ReplayDataset(&system, source), 2);
  EXPECT_TRUE(system.initialized());
  EXPECT_NEAR(system.state().pose.translation().x(), 0.5, 1e-9);

  const std::filesystem::path trajectory = directory / "trajectory.txt";
  ASSERT_TRUE(system.WriteTrajectory(trajectory.string()));
  std::ifstream written(trajectory);
  std::string line;
  ASSERT_TRUE(static_cast<bool>(std::getline(written, line)));
  EXPECT_FALSE(line.empty());

  std::filesystem::remove(directory / "scans" / "000001.txt");
  EXPECT_FALSE(source.Load(directory.string()));
  std::filesystem::remove_all(directory);
}

}  // namespace
}  // namespace app
}  // namespace argus
