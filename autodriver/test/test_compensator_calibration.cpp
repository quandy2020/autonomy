/*
 * Copyright 2026 Autodriver contributors
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

#include <gtest/gtest.h>

#include <cmath>
#include <cstdio>
#include <cstring>
#include <fstream>
#include <string>

#include <Eigen/Geometry>

#include "autodriver/common/calibration.hpp"
#include "autodriver/lidar/hesai/calibration.hpp"
#include "autodriver/lidar/motion_compensator.hpp"
#include "autodriver/lidar/motion_pose_sink.hpp"
#include "autodriver/lidar/pose_buffer.hpp"
#include "autodriver/lidar/velodyne/calibration.hpp"
#include "autodriver/lidar/velodyne/udp_driver.hpp"

namespace {

automsgs::msgs::sensor_msgs::PointCloud2 MakeCloudWithTimestamps(
    float x0, float x1, double t0_ns, double t1_ns) {
    automsgs::msgs::sensor_msgs::PointCloud2 cloud;
    cloud.mutable_header()->set_frame_id("velodyne");
    cloud.set_height(1);
    cloud.set_width(2);
    cloud.set_is_dense(true);
    cloud.set_point_step(24);
    cloud.set_row_step(48);
    using PF = automsgs::msgs::sensor_msgs::PointField;
    auto add = [&](const char* name, std::uint32_t offset, PF::DataType dt) {
        auto* f = cloud.add_fields();
        f->set_name(name);
        f->set_offset(offset);
        f->set_datatype(dt);
        f->set_count(1);
    };
    add("x", 0, PF::FLOAT32);
    add("y", 4, PF::FLOAT32);
    add("z", 8, PF::FLOAT32);
    add("intensity", 12, PF::FLOAT32);
    add("timestamp", 16, PF::FLOAT64);

    std::string data(48, '\0');
    float y = 0;
    float z = 0;
    float i = 1;
    std::memcpy(&data[0], &x0, 4);
    std::memcpy(&data[4], &y, 4);
    std::memcpy(&data[8], &z, 4);
    std::memcpy(&data[12], &i, 4);
    std::memcpy(&data[16], &t0_ns, 8);
    std::memcpy(&data[24], &x1, 4);
    std::memcpy(&data[28], &y, 4);
    std::memcpy(&data[32], &z, 4);
    std::memcpy(&data[36], &i, 4);
    std::memcpy(&data[40], &t1_ns, 8);
    *cloud.mutable_data() = std::move(data);
    return cloud;
}

}  // namespace

TEST(Calibration, LoadExtrinsicYamlShape) {
    const std::string path = "/tmp/autodriver_test_extrinsics.yaml";
    {
        std::ofstream ofs(path);
        ASSERT_TRUE(ofs.is_open());
        ofs << R"(
header:
  frame_id: novatel
child_frame_id: velodyne
transform:
  translation:
    x: 1.0
    y: 2.0
    z: 3.0
  rotation:
    x: 0.0
    y: 0.0
    z: 0.0
    w: 1.0
)";
    }
    autodriver::common::Extrinsic ext;
    std::string err;
    ASSERT_TRUE(autodriver::common::LoadExtrinsicYaml(path, &ext, &err)) << err;
    EXPECT_EQ(ext.parent_frame, "novatel");
    EXPECT_EQ(ext.child_frame, "velodyne");
    EXPECT_NEAR(ext.transform.translation().x(), 1.0, 1e-9);
    EXPECT_NEAR(ext.transform.translation().y(), 2.0, 1e-9);
    EXPECT_NEAR(ext.transform.translation().z(), 3.0, 1e-9);
    std::remove(path.c_str());
}

TEST(MotionCompensator, PureTranslationToScanEnd) {
    const std::uint64_t t0 = 1'000'000'000ULL;
    const std::uint64_t t1 = 1'100'000'000ULL;
    Eigen::Affine3d pose_min = Eigen::Translation3d(0, 0, 0) *
                               Eigen::Quaterniond::Identity();
    Eigen::Affine3d pose_max = Eigen::Translation3d(1, 0, 0) *
                               Eigen::Quaterniond::Identity();

    autodriver::lidar::MotionCompensator compensator;
    compensator.SetPoseLookup(autodriver::lidar::MakeLinearPoseLookup(
        t0, t1, pose_min, pose_max));

    // Point at t0 should move by ~1m toward scan-end frame.
    auto in = MakeCloudWithTimestamps(0.0f, 0.0f, static_cast<double>(t0),
                                      static_cast<double>(t1));
    automsgs::msgs::sensor_msgs::PointCloud2 out;
    ASSERT_TRUE(compensator.Compensate(in, &out));
    float x0 = 0;
    float x1 = 0;
    std::memcpy(&x0, out.data().data(), 4);
    std::memcpy(&x1, out.data().data() + 24, 4);
    EXPECT_NEAR(x0, 1.0f, 1e-3f);
    EXPECT_NEAR(x1, 0.0f, 1e-3f);
}

TEST(PoseBuffer, InterpolatesBetweenPoses) {
    autodriver::lidar::PoseBuffer buffer(10);
    Eigen::Affine3d a = Eigen::Translation3d(0, 0, 0) *
                        Eigen::Quaterniond::Identity();
    Eigen::Affine3d b = Eigen::Translation3d(2, 0, 0) *
                        Eigen::Quaterniond::Identity();
    buffer.Push(1000, a);
    buffer.Push(3000, b);
    auto lookup = buffer.AsLookup();
    Eigen::Affine3d mid;
    ASSERT_TRUE(lookup(2000, "lidar", &mid));
    EXPECT_NEAR(mid.translation().x(), 1.0, 1e-6);
}

TEST(VelodyneCalibration, LoadBeamYaml) {
    const std::string path = "/tmp/autodriver_vlp16_cal.yaml";
    {
        std::ofstream ofs(path);
        ASSERT_TRUE(ofs.is_open());
        ofs << "lasers:\n"
               "- {laser_id: 0, vert_correction: -0.26}\n"
               "- {laser_id: 1, vert_correction: 0.02}\n";
    }
    autodriver::lidar::velodyne::BeamCalibration cal;
    std::string err;
    ASSERT_TRUE(
        autodriver::lidar::velodyne::LoadBeamCalibrationYaml(path, &cal, &err))
        << err;
    ASSERT_GE(cal.vert_correction_rad.size(), 2u);
    EXPECT_NEAR(cal.vert_correction_rad[0], -0.26, 1e-9);
    EXPECT_NEAR(cal.vert_correction_rad[1], 0.02, 1e-9);
    std::remove(path.c_str());
}

TEST(HesaiCalibration, LoadBeamYamlDegrees) {
    const std::string path = "/tmp/autodriver_xt32_cal.yaml";
    {
        std::ofstream ofs(path);
        ASSERT_TRUE(ofs.is_open());
        ofs << "lasers:\n";
        for (int i = 0; i < 32; ++i) {
            ofs << "- {laser_id: " << i << ", vert_correction: " << (15 - i)
                << "}\n";
        }
    }
    autodriver::lidar::hesai::BeamCalibration cal;
    std::string err;
    ASSERT_TRUE(
        autodriver::lidar::hesai::LoadBeamCalibrationYaml(path, &cal, &err))
        << err;
    EXPECT_NEAR(cal.elev_deg[0], 15.0, 1e-9);
    EXPECT_NEAR(cal.elev_deg[12], 3.0, 1e-9);  // must stay degrees, not rad→deg
    EXPECT_NEAR(cal.elev_deg[31], -16.0, 1e-9);
    std::remove(path.c_str());
}

TEST(HesaiCalibration, LoadBeamYamlRadiansFlag) {
    const std::string path = "/tmp/autodriver_xt32_cal_rad.yaml";
    {
        std::ofstream ofs(path);
        ASSERT_TRUE(ofs.is_open());
        ofs << "unit: rad\nlasers:\n";
        for (int i = 0; i < 32; ++i) {
            const double deg = static_cast<double>(15 - i);
            constexpr double kDegToRad = 3.14159265358979323846 / 180.0;
            ofs << "- {laser_id: " << i
                << ", vert_correction: " << (deg * kDegToRad) << "}\n";
        }
    }
    autodriver::lidar::hesai::BeamCalibration cal;
    std::string err;
    ASSERT_TRUE(
        autodriver::lidar::hesai::LoadBeamCalibrationYaml(path, &cal, &err))
        << err;
    EXPECT_NEAR(cal.elev_deg[0], 15.0, 1e-6);
    EXPECT_NEAR(cal.elev_deg[31], -16.0, 1e-6);
    std::remove(path.c_str());
}

TEST(MotionPoseSink, VelodyneDriverAcceptsPushPose) {
    autodriver::hardware::DriverParams params;
    params["source_type"] = "raw_packet";
    params["enable_compensator"] = "true";
    params["packets_per_scan"] = "1";
    auto driver =
        autodriver::hardware::CreateVelodyneUdpDriver("lidar/vlp16", params);
    ASSERT_NE(driver, nullptr);
    auto* sink =
        dynamic_cast<autodriver::lidar::MotionPoseSink*>(driver.get());
    ASSERT_NE(sink, nullptr);
    ASSERT_NE(sink->pose_buffer(), nullptr);
    Eigen::Affine3d pose = Eigen::Affine3d::Identity();
    pose.translation() = Eigen::Vector3d(1, 2, 3);
    sink->PushPose(1'000'000'000ULL, pose);
    Eigen::Affine3d got;
    ASSERT_TRUE(sink->pose_buffer()->AsLookup()(1'000'000'000ULL, "lidar/vlp16",
                                                &got));
    EXPECT_NEAR(got.translation().x(), 1.0, 1e-9);
}
