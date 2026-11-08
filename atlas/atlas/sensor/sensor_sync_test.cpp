/*
 * Copyright 2026 The Openbot Authors
 */

#include "sensor/sensor_sync.hpp"

#include "gtest/gtest.h"

namespace atlas {
namespace sensor {
namespace {

TEST(SensorSync, RestoresPendingScanIntoNewMeasureGroupAfterWaitingForImu) {
  SensorSync sensor_sync;
  auto point_cloud = std::make_shared<PointCloudXYZI>();
  point_cloud->points.resize(2);
  point_cloud->points.back().curvature = 0.1F;
  sensor_sync.PushLidar(point_cloud, 10.0);

  ImuData first_imu;
  first_imu.time = 10.05;
  sensor_sync.PushImu(first_imu);

  LidarMeasureGroup first_measure_group;
  EXPECT_FALSE(sensor_sync.Sync(first_measure_group));

  ImuData final_imu;
  final_imu.time = 10.11;
  sensor_sync.PushImu(final_imu);
  LidarMeasureGroup second_measure_group;
  ASSERT_TRUE(sensor_sync.Sync(second_measure_group));

  EXPECT_EQ(second_measure_group.lidar, point_cloud);
  EXPECT_DOUBLE_EQ(second_measure_group.lidar_frame_beg_time, 10.0);
  EXPECT_NEAR(second_measure_group.lidar_frame_end_time, 10.1, 1e-7);
  ASSERT_EQ(second_measure_group.measures.size(), 1U);
  EXPECT_NEAR(second_measure_group.measures.front().lio_time, 10.1, 1e-7);
}

}  // namespace
}  // namespace sensor
}  // namespace atlas
