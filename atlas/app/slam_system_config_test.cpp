#include "slam_system_config.hpp"

#include <gtest/gtest.h>

#include <fstream>

namespace atlas {
namespace app {
namespace {

TEST(SlamSystemConfig, DefaultsAreValid) {
  const SlamSystemConfig config = DefaultSlamSystemConfig();
  EXPECT_EQ(config.pipeline, "lidar");
  EXPECT_EQ(config.min_imu_samples, 20);
  EXPECT_DOUBLE_EQ(config.gravity_magnitude, 9.81);
  EXPECT_EQ(config.window_size, 10u);
  ASSERT_EQ(config.components.size(), 1u);
  EXPECT_EQ(config.components[0].name, "lidar_frontend");
  EXPECT_EQ(config.components[0].type, "lidar_frontend");
  EXPECT_TRUE(ValidateSlamSystemConfig(config));
}

TEST(SlamSystemConfig, RoundTripKeepsNestedFields) {
  SlamSystemConfig config = DefaultSlamSystemConfig();
  config.pipeline = "visual";
  config.min_imu_samples = 4;
  config.gravity_magnitude = 9.81;
  config.max_accel_variance = 0.2;
  config.window_size = 3;
  config.voxel_bias_gyro_random_walk = 2e-4;
  config.voxel_bias_accel_random_walk = 3e-4;
  config.voxel_minimum_bundle_adjustment_points = 24;
  config.voxel_gba_voxel_size = 1.5;
  config.voxel_gba_minimum_eigenvalue = 0.02;
  config.voxel_gba_maximum_iteration_count = 4;
  config.trajectory_path = "/tmp/traj.txt";
  config.components = {{"front", "lidar_frontend"},
                       {"vision", "visual_frontend"}};
  const std::string path = "slam_system_config_round_trip.yaml";
  ASSERT_TRUE(SaveSlamSystemConfig(config, path));

  SlamSystemConfig loaded;
  ASSERT_TRUE(LoadSlamSystemConfig(path, &loaded));
  EXPECT_EQ(loaded.pipeline, config.pipeline);
  EXPECT_EQ(loaded.min_imu_samples, 4);
  EXPECT_DOUBLE_EQ(loaded.gravity_magnitude, config.gravity_magnitude);
  EXPECT_DOUBLE_EQ(loaded.max_accel_variance, config.max_accel_variance);
  EXPECT_EQ(loaded.window_size, 3u);
  EXPECT_DOUBLE_EQ(loaded.voxel_bias_gyro_random_walk, 2e-4);
  EXPECT_DOUBLE_EQ(loaded.voxel_bias_accel_random_walk, 3e-4);
  EXPECT_EQ(loaded.voxel_minimum_bundle_adjustment_points, 24);
  EXPECT_DOUBLE_EQ(loaded.voxel_gba_voxel_size, 1.5);
  EXPECT_DOUBLE_EQ(loaded.voxel_gba_minimum_eigenvalue, 0.02);
  EXPECT_EQ(loaded.voxel_gba_maximum_iteration_count, 4);
  EXPECT_EQ(loaded.trajectory_path, config.trajectory_path);
  ASSERT_EQ(loaded.components.size(), 2u);
  EXPECT_EQ(loaded.components[1].name, "vision");
  EXPECT_EQ(loaded.components[1].type, "visual_frontend");
}

TEST(SlamSystemConfig, RejectsUnknownKeyAndDuplicateName) {
  const std::string unknown = "slam_system_config_unknown.yaml";
  {
    std::ofstream output(unknown);
    output << "foo: 1\n";
  }
  SlamSystemConfig config;
  EXPECT_FALSE(LoadSlamSystemConfig(unknown, &config));

  const std::string duplicate = "slam_system_config_duplicate.yaml";
  {
    std::ofstream output(duplicate);
    output << "components:\n";
    output << "  - name: front\n";
    output << "    type: lidar_frontend\n";
    output << "  - name: front\n";
    output << "    type: visual_frontend\n";
  }
  EXPECT_FALSE(LoadSlamSystemConfig(duplicate, &config));
  EXPECT_FALSE(LoadSlamSystemConfig("missing_slam_system.yaml", &config));
  EXPECT_FALSE(ValidateSlamSystemConfig(
      SlamSystemConfig{"lidar", 0, 9.81, 0.05, 10, {}, {}}));
}

}  // namespace
}  // namespace app
}  // namespace atlas
