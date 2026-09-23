/*
 * Copyright 2026 The Openbot Authors
 *
 * AtlasConfig Load/Save roundtrip (ORB-SLAM3 Settings coverage).
 */

/**
 * @file config_test.cpp
 * @brief AtlasConfig YAML round-trip unit tests (ORB-SLAM3 Settings fields).
 */

#include "autonomy/localization/atlas/common/config.hpp"

#include <cstdio>
#include <fstream>
#include <sstream>
#include <string>

#include "gtest/gtest.h"

namespace autonomy {
namespace localization {
namespace atlas {
namespace {

std::string TempPath(const std::string& name) {
    return std::string("/tmp/atlas_cfg_") + name + ".yaml";
}

TEST(AtlasConfigTest, SaveLoadRoundtrip) {
    AtlasConfig cfg;
    cfg.platform_name = "roundtrip";
    cfg.mode = FrontendMode::kVio;
    cfg.fusion = FusionStyle::kTight;
    cfg.vio_enabled = true;
    cfg.camera_type = CameraModelType::kPinhole;
    cfg.camera_fx = 458.654;
    cfg.camera_fy = 457.296;
    cfg.camera_cx = 367.215;
    cfg.camera_cy = 248.375;
    cfg.camera_distortion = {-0.28, 0.07, 0.0, 0.0};
    cfg.image_width = 752;
    cfg.image_height = 480;
    cfg.fps = 20.f;
    cfg.orb.num_features = 1000;
    cfg.orb.scale_factor = 1.2f;
    cfg.orb.num_levels = 8;
    cfg.imu_gyro_noise = 1.7e-4;
    cfg.imu_accel_noise = 2e-3;
    cfg.imu_frequency = 200.f;
    cfg.T_b_c.assign(16, 0.0);
    cfg.T_b_c[0] = cfg.T_b_c[5] = cfg.T_b_c[10] = cfg.T_b_c[15] = 1.0;
    cfg.vocabulary_path = "vocab.fbow";
    cfg.backend = BackendType::kCeres;

    const std::string path = TempPath("roundtrip");
    ASSERT_TRUE(SaveConfig(path, cfg));

    AtlasConfig loaded;
    ASSERT_TRUE(LoadConfig(path, &loaded));
    EXPECT_EQ(loaded.platform_name, "roundtrip");
    EXPECT_DOUBLE_EQ(loaded.camera_fx, 458.654);
    EXPECT_EQ(loaded.orb.num_features, 1000);
    EXPECT_FLOAT_EQ(loaded.orb.scale_factor, 1.2f);
    EXPECT_EQ(loaded.camera_distortion.size(), 4u);
    EXPECT_EQ(loaded.T_b_c.size(), 16u);
    EXPECT_DOUBLE_EQ(loaded.imu_gyro_noise, 1.7e-4);
    EXPECT_EQ(loaded.vocabulary_path, "vocab.fbow");
    std::remove(path.c_str());
}

TEST(AtlasConfigTest, LoadOrbFlatSchema) {
    const std::string path = TempPath("orb_flat");
    {
        std::ofstream ofs(path);
        ofs << "Camera.type: \"PinHole\"\n"
            << "Camera1.fx: 458.654\n"
            << "Camera1.fy: 457.296\n"
            << "Camera1.cx: 367.215\n"
            << "Camera1.cy: 248.375\n"
            << "Camera1.k1: -0.28\n"
            << "Camera1.k2: 0.07\n"
            << "Camera1.p1: 0.0\n"
            << "Camera1.p2: 0.0\n"
            << "Camera.width: 752\n"
            << "Camera.height: 480\n"
            << "Camera.fps: 20\n"
            << "Camera.RGB: 1\n"
            << "IMU.NoiseGyro: 1.7e-4\n"
            << "IMU.NoiseAcc: 2.0e-3\n"
            << "IMU.GyroWalk: 1.9e-5\n"
            << "IMU.AccWalk: 3.0e-3\n"
            << "IMU.Frequency: 200.0\n"
            << "ORBextractor.nFeatures: 1000\n"
            << "ORBextractor.scaleFactor: 1.2\n"
            << "ORBextractor.nLevels: 8\n"
            << "ORBextractor.iniThFAST: 20\n"
            << "ORBextractor.minThFAST: 7\n";
    }
    AtlasConfig cfg;
    ASSERT_TRUE(LoadConfig(path, &cfg));
    EXPECT_EQ(cfg.schema, "orb");
    EXPECT_DOUBLE_EQ(cfg.camera_fx, 458.654);
    EXPECT_EQ(cfg.orb.num_features, 1000);
    EXPECT_EQ(cfg.camera_distortion.size(), 4u);
    EXPECT_TRUE(cfg.need_undistort);
    EXPECT_DOUBLE_EQ(cfg.imu_gyro_noise, 1.7e-4);
    std::remove(path.c_str());
}

TEST(AtlasConfigTest, DumpContainsSections) {
    AtlasConfig cfg;
    cfg.platform_name = "dump";
    cfg.orb.num_features = 999;
    std::ostringstream oss;
    DumpConfig(oss, cfg);
    const std::string s = oss.str();
    EXPECT_NE(s.find("ORB:"), std::string::npos);
    EXPECT_NE(s.find("999"), std::string::npos);
    EXPECT_NE(s.find("IMU:"), std::string::npos);
}

}  // namespace
}  // namespace atlas
}  // namespace localization
}  // namespace autonomy
