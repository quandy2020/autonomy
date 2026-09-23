/*
 * Copyright 2026 The Openbot Authors
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 */

/**
 * @file tracking_test.cpp
 * @brief Frontend factory and SlamSystem smoke tests (VO / dry run).
 */

#include "autonomy/localization/atlas/frontend/factory.hpp"
#include "autonomy/localization/atlas/map/map.hpp"
#include "autonomy/localization/atlas/system/slam_system.hpp"

#include "gtest/gtest.h"

namespace autonomy {
namespace localization {
namespace atlas {
namespace {

TEST(AtlasFactoryTest, CreatesVisualOdometry) {
  AtlasConfig config;
  config.mode = FrontendMode::kVo;
  auto frontend = CreateFrontend(config);
  ASSERT_NE(frontend, nullptr);
  EXPECT_EQ(frontend->Mode(), FrontendMode::kVo);
}

TEST(AtlasFactoryTest, VioSensorFollowsCameraRig) {
  EXPECT_EQ(tracking::Tracker::SensorFromConfig(
                [] {
                  AtlasConfig c;
                  c.camera_sensor = CameraSensor::kMonocular;
                  return c;
                }(),
                true),
            tracking::Tracker::Sensor::kImuMonocular);
  EXPECT_EQ(tracking::Tracker::SensorFromConfig(
                [] {
                  AtlasConfig c;
                  c.camera_sensor = CameraSensor::kStereo;
                  return c;
                }(),
                true),
            tracking::Tracker::Sensor::kImuStereo);
  EXPECT_EQ(tracking::Tracker::SensorFromConfig(
                [] {
                  AtlasConfig c;
                  c.camera_sensor = CameraSensor::kRgbd;
                  return c;
                }(),
                true),
            tracking::Tracker::Sensor::kImuRgbd);
}

TEST(AtlasFactoryTest, CreatesVisualInertial) {
  AtlasConfig config;
  config.mode = FrontendMode::kVio;
  auto frontend = CreateFrontend(config);
  ASSERT_NE(frontend, nullptr);
  EXPECT_EQ(frontend->Mode(), FrontendMode::kVio);
}

TEST(MapTest, StartsEmpty) {
  Map map;
  EXPECT_EQ(map.MapPointsInMap(), 0u);
  EXPECT_EQ(map.KeyFramesInMap(), 0u);
}

TEST(SlamSystemTest, InitRgbd) {
  SlamSystem system;
  AtlasConfig config;
  ASSERT_TRUE(system.Init(config, SlamSystem::Sensor::kRgbd));
  EXPECT_NE(system.mutable_map(), nullptr);
}

}  // namespace
}  // namespace atlas
}  // namespace localization
}  // namespace autonomy
