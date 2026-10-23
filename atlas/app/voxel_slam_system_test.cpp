#include "voxel_slam_system.hpp"

#include <gtest/gtest.h>

#include <cmath>
#include <filesystem>

namespace atlas {
namespace app {
namespace {

sensor::PointCloudXYZI MakeRoomScan() {
  sensor::PointCloudXYZI cloud;
  int point_index = 0;
  constexpr int kSteps = 33;
  constexpr float kHalfExtent = 4.0f;
  constexpr float kWall = 5.0f;
  auto add = [&](float x, float y, float z) {
    sensor::PointType point;
    point.x = x;
    point.y = y;
    point.z = z;
    point.intensity = 1.0f;
    point.curvature = 100.0f * static_cast<float>(point_index++) /
                      static_cast<float>(6 * kSteps * kSteps - 1);
    cloud.push_back(point);
  };
  for (int i = 0; i < kSteps; ++i) {
    const float a = -kHalfExtent + 2.0f * kHalfExtent * i / (kSteps - 1);
    for (int j = 0; j < kSteps; ++j) {
      const float b = -kHalfExtent + 2.0f * kHalfExtent * j / (kSteps - 1);
      add(-kWall, a, b);
      add(kWall, a, b);
      add(a, -kWall, b);
      add(a, kWall, b);
      add(a, b, -kWall);
      add(a, b, kWall);
    }
  }
  return cloud;
}

SlamSystemConfig MakeVoxelTestConfig(const std::filesystem::path& directory) {
  SlamSystemConfig config = DefaultSlamSystemConfig();
  config.pipeline = "voxel_slam";
  config.components.clear();
  config.voxel_win_size = 3;
  config.voxel_thread_num = 1;
  config.voxel_down_size = 0.18;
  config.voxel_size = 1.0;
  config.voxel_enable_loop = false;
  config.voxel_enable_gba = false;
  config.voxel_enable_local_ba = true;
  config.voxel_enable_triangle = false;
  config.voxel_save_path = directory.string();
  config.voxel_bagname = "synthetic_room";
  return config;
}

TEST(VoxelSlamSystem, InitializesAndProcessesSyntheticRoom) {
  const std::filesystem::path directory =
      std::filesystem::temp_directory_path() / "atlas_voxel_system_e2e";
  std::filesystem::remove_all(directory);
  std::filesystem::create_directories(directory);

  VoxelSlamSystem system(MakeVoxelTestConfig(directory));
  ASSERT_TRUE(system.Start());
  const sensor::PointCloudXYZI cloud = MakeRoomScan();
  ASSERT_GT(cloud.size(), 1000u);

  constexpr double kImuStep = 0.005;
  double imu_time = 0.0;
  for (int scan_index = 0; scan_index < 10; ++scan_index) {
    const double scan_time = 0.2 * scan_index;
    const double scan_end = scan_time + 0.1;
    while (imu_time <= scan_end + kImuStep) {
      sensor::ImuData imu;
      imu.time = imu_time;
      imu.linear_acceleration = Eigen::Vector3d(0.0, 0.0, 9.81);
      imu.angular_velocity.setZero();
      system.AddImu(imu);
      imu_time += kImuStep;
    }
    ASSERT_TRUE(system.AddScan(cloud, scan_time));
  }

  ASSERT_TRUE(system.initialized());
  ASSERT_FALSE(system.trajectory().empty());
  EXPECT_TRUE(system.trajectory().back().p.allFinite());
  EXPECT_NEAR(system.trajectory().back().g.norm(), 9.81, 0.2);

  std::string error;
  EXPECT_TRUE(system.Stop(&error)) << error;
  std::filesystem::remove_all(directory);
}

}  // namespace
}  // namespace app
}  // namespace atlas
