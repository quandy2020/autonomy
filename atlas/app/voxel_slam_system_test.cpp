#include "voxel_slam_system.hpp"

#include <gtest/gtest.h>

namespace atlas {
namespace app {
namespace {

SlamSystemConfig MakeTestConfiguration() {
  SlamSystemConfig config = DefaultSlamSystemConfig();
  config.pipeline = "voxel_slam";
  config.voxel_slam.lidar.point_filter_stride = 1;
  config.voxel_slam.global_mapping.enabled = false;
  return config;
}

sensor::PointCloudXYZI MakeTimedScan() {
  sensor::PointCloudXYZI cloud;
  sensor::PointType first_point;
  first_point.x = 1.0f;
  first_point.curvature = 0.0f;
  sensor::PointType last_point = first_point;
  last_point.y = 1.0f;
  last_point.curvature = 0.1f;
  cloud.push_back(first_point);
  cloud.push_back(last_point);
  return cloud;
}

void AddImuSamples(atlas::app::VoxelSlamSystem* system, int first_sample,
                   int last_sample) {
  for (int sample_index = first_sample; sample_index <= last_sample;
       ++sample_index) {
    sensor::ImuData imu;
    imu.time = static_cast<double>(sample_index) * 0.0025;
    imu.linear_acceleration = Eigen::Vector3d(0.0, 0.0, 9.81);
    system->AddImu(imu);
  }
}

TEST(VoxelSlamSystem, RejectsRepeatedStartAndAllowsOneFinalization) {
  VoxelSlamSystem system(MakeTestConfiguration());

  ASSERT_TRUE(system.Start());
  EXPECT_FALSE(system.Start());
  EXPECT_TRUE(system.started());
  EXPECT_TRUE(system.Stop());
  EXPECT_FALSE(system.started());
  EXPECT_FALSE(system.Start());
}

TEST(VoxelSlamSystem, StopBeforeStartIsHarmless) {
  VoxelSlamSystem system(MakeTestConfiguration());

  EXPECT_TRUE(system.Stop());
  EXPECT_FALSE(system.started());
}

TEST(VoxelSlamSystem, DoesNotRecordPosesBeforeMotionInitializationCompletes) {
  VoxelSlamSystem system(MakeTestConfiguration());
  ASSERT_TRUE(system.Start());
  auto& builder = system.trajectory_builder();
  ::atlas::odometry::OdometryOutput output;

  AddImuSamples(&system, 0, 42);
  builder.AddLidar(std::make_shared<sensor::PointCloudXYZI>(MakeTimedScan()),
                   0.0);
  ASSERT_TRUE(builder.Process(&output));
  EXPECT_EQ(output.status, ::atlas::odometry::ProcessStatus::kPropagateOnly);

  AddImuSamples(&system, 43, 82);
  builder.AddLidar(std::make_shared<sensor::PointCloudXYZI>(MakeTimedScan()),
                   0.1);
  ASSERT_TRUE(builder.Process(&output));

  EXPECT_EQ(output.status, ::atlas::odometry::ProcessStatus::kPropagateOnly);
  EXPECT_TRUE(system.Stop());
}

TEST(VoxelSlamSystem, RejectsInvalidAlgorithmConfiguration) {
  ::atlas::frontend::TrajectoryBuilderOptions configuration;
  configuration.local_window_size = 0;
  ::atlas::frontend::TrajectoryBuilder trajectory_builder(configuration);

  trajectory_builder.Initialize();
  EXPECT_FALSE(trajectory_builder.initialized());
}

TEST(VoxelSlamSystem, OdometryInitializationIsNotRepeated) {
  ::atlas::frontend::TrajectoryBuilderOptions configuration;
  configuration.global_bundle_adjustment_enabled = true;
  ::atlas::frontend::TrajectoryBuilder trajectory_builder(configuration);

  trajectory_builder.Initialize();
  ASSERT_TRUE(trajectory_builder.initialized());
  trajectory_builder.Initialize();
  EXPECT_TRUE(trajectory_builder.initialized());
  EXPECT_TRUE(trajectory_builder.Finalize());
}

}  // namespace
}  // namespace app
}  // namespace atlas
