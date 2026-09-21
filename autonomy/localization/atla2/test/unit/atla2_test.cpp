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

#include "gtest/gtest.h"

#include "autonomy/localization/atla2/common/config.hpp"
#include "autonomy/localization/atla2/common/time.hpp"
#include "autonomy/localization/atla2/frontend/factory.hpp"
#include "autonomy/localization/atla2/pipeline/slam_system.hpp"
#include "autonomy/localization/atla2/proto/convert/sensor_packet_codec.hpp"

namespace autonomy::localization::atla2 {
namespace {

TEST(Atla2Config, ParseModeDefaults) {
  EXPECT_EQ(ParseFrontendMode("vo"), FrontendMode::kVo);
  EXPECT_EQ(ParseFrontendMode("vio"), FrontendMode::kVio);
  EXPECT_EQ(ParseFrontendMode("lo"), FrontendMode::kLo);
  EXPECT_EQ(ParseFrontendMode("lio"), FrontendMode::kLio);
  EXPECT_EQ(ParseFrontendMode("livo"), FrontendMode::kLivo);
  EXPECT_EQ(ToString(FrontendMode::kVo), "vo");
  EXPECT_EQ(ToString(FrontendMode::kLo), "lo");
  EXPECT_EQ(ToString(FrontendMode::kLio), "lio");
}

TEST(Atla2Factory, CreateVioLioLivo) {
  Atla2Config cfg;
  cfg.mode = FrontendMode::kVo;
  auto vo = CreateFrontend(cfg);
  ASSERT_NE(vo, nullptr);
  EXPECT_EQ(vo->Mode(), FrontendMode::kVo);

  cfg.mode = FrontendMode::kVio;
  auto vio = CreateFrontend(cfg);
  ASSERT_NE(vio, nullptr);
  EXPECT_EQ(vio->Mode(), FrontendMode::kVio);

  cfg.mode = FrontendMode::kLo;
  auto lo = CreateFrontend(cfg);
  ASSERT_NE(lo, nullptr);
  EXPECT_EQ(lo->Mode(), FrontendMode::kLo);

  cfg.mode = FrontendMode::kLio;
  auto lio = CreateFrontend(cfg);
  ASSERT_NE(lio, nullptr);
  EXPECT_EQ(lio->Mode(), FrontendMode::kLio);

  cfg.mode = FrontendMode::kLivo;
  auto livo = CreateFrontend(cfg);
  ASSERT_NE(livo, nullptr);
  EXPECT_EQ(livo->Mode(), FrontendMode::kLivo);
}

TEST(Atla2Backend, CeresVioWindow) {
  Atla2Config cfg;
  cfg.mode = FrontendMode::kVio;
  cfg.backend = BackendType::kCeres;
  cfg.window_size = 5;
  cfg.ceres_max_iterations = 5;

  SlamSystem slam;
  ASSERT_TRUE(slam.Init(cfg));

  for (int k = 0; k < 8; ++k) {
    SensorData data;
    data.t = SecToStamp(k * 0.05);
    data.has_imu = true;
    for (int j = 0; j < 5; ++j) {
      ImuSample s;
      s.t = SecToStamp(k * 0.05 - 0.04 + j * 0.01);
      s.accel = Vec3(0, 0, 9.81);
      s.gyro = Vec3(0, 0, 0.02);
      data.imu.push_back(s);
    }
    data.has_image = true;
    data.image.t = data.t;
    data.image.width = 640;
    data.image.height = 480;
    ASSERT_TRUE(slam.Step(data));
  }
  OdometryResult odom;
  ASSERT_TRUE(slam.GetOdometry(&odom));
  EXPECT_TRUE(odom.valid);
}

TEST(Atla2SlamSystem, SyntheticLioSteps) {
  Atla2Config cfg;
  cfg.mode = FrontendMode::kLio;
  cfg.max_local_map_points = 5000;
  cfg.voxel_size = 0.5;

  SlamSystem slam;
  ASSERT_TRUE(slam.Init(cfg));

  for (int k = 0; k < 20; ++k) {
    SensorData data;
    data.t = SecToStamp(k * 0.1);
    data.has_imu = true;
    for (int j = 0; j < 3; ++j) {
      ImuSample s;
      s.t = SecToStamp(k * 0.1 - 0.02 + j * 0.01);
      s.accel = Vec3(0, 0, 9.81);
      s.gyro = Vec3::Zero();
      data.imu.push_back(s);
    }
    data.has_lidar = true;
    data.lidar.t = data.t;
    PointXYZI p;
    p.x = 1.f;
    p.y = 0.f;
    p.z = 0.f;
    data.lidar.points.push_back(p);
    ASSERT_TRUE(slam.Step(data));
  }

  OdometryResult odom;
  ASSERT_TRUE(slam.GetOdometry(&odom));
  EXPECT_TRUE(odom.valid);
  EXPECT_FALSE(slam.map()->Keyframes().empty());
}

TEST(Atla2ProtoCodec, SensorDataRoundTrip) {
  SensorData in;
  in.t = SecToStamp(1.25);
  in.has_imu = true;
  ImuSample s;
  s.t = in.t;
  s.accel = Vec3(0, 0, 9.81);
  s.gyro = Vec3(0.01, 0, 0);
  in.imu.push_back(s);
  in.has_image = true;
  in.image.t = in.t;
  in.image.width = 4;
  in.image.height = 2;
  in.image.channels = 1;
  in.image.frame_id = "cam";
  in.image.data = {1, 2, 3, 4, 5, 6, 7, 8};
  in.has_lidar = true;
  in.lidar.t = in.t;
  in.lidar.t_begin = SecToStamp(1.15);
  in.lidar.model = "mid360";
  PointXYZI p;
  p.x = 1.f;
  p.y = 2.f;
  p.z = 3.f;
  p.intensity = 0.5f;
  p.timestamp = 1.2;
  in.lidar.points.push_back(p);
  in.has_gps = true;
  in.gps.valid = true;
  in.gps.lla = Vec3(31.0, 121.0, 10.0);
  in.gps.position_enu = Vec3(1, 2, 3);
  in.gps.hdop = 1.5;
  in.has_baro = true;
  in.baro.valid = true;
  in.baro.pressure_pa = 101325.0;
  in.baro.altitude_m = 5.0;
  in.has_optical_flow = true;
  in.optical_flow.valid = true;
  in.optical_flow.flow_px = Vec2(1.0, -1.0);
  in.optical_flow.quality = 0.9;

  SyncedSensorPacketDto dto;
  ASSERT_TRUE(SensorDataToDto(in, &dto));
  SensorData out;
  ASSERT_TRUE(DtoToSensorData(dto, &out));
  EXPECT_TRUE(SensorDataRoundTripEqual(in, out));
}

TEST(Atla2HotSwitch, LivoDegradesToLio) {
  Atla2Config cfg;
  cfg.mode = FrontendMode::kLivo;
  cfg.enable_mode_hot_switch = true;
  cfg.backend = BackendType::kIekf;

  SlamSystem slam;
  ASSERT_TRUE(slam.Init(cfg));
  EXPECT_EQ(slam.active_mode(), FrontendMode::kLivo);

  SensorData data;
  data.t = SecToStamp(0.1);
  data.has_imu = true;
  ImuSample s;
  s.t = data.t;
  s.accel = Vec3(0, 0, 9.81);
  data.imu.push_back(s);
  data.has_lidar = true;
  data.lidar.t = data.t;
  PointXYZI p;
  p.x = 1.f;
  data.lidar.points.push_back(p);
  // No image -> LIVO degrades toward LIO/LO.
  ASSERT_TRUE(slam.Step(data));
  EXPECT_EQ(slam.state(), SlamState::kDegraded);
  EXPECT_TRUE(slam.active_mode() == FrontendMode::kLio ||
              slam.active_mode() == FrontendMode::kLo);
}

}  // namespace
}  // namespace autonomy::localization::atla2
