/*
 * Copyright 2026 The OpenRobotic Beginner Authors (duyongquan)
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

/**
 * @file grid_test.cpp
 * @brief Contract tests for the rolling Shadow 2.5D safety grid.
 */

#include "autonomy/perception/shadow/grid.hpp"

#include <gtest/gtest.h>

#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <limits>
#include <string>
#include <vector>

namespace autonomy {
namespace perception {
namespace shadow {
namespace {

using CameraInfo = automsgs::msgs::sensor_msgs::CameraInfo;
using GridMapMessage = automsgs::msgs::map_msgs::GridMap;
using Image = automsgs::msgs::sensor_msgs::Image;
using Odometry = automsgs::msgs::nav_msgs::Odometry;
using TransformStamped = automsgs::msgs::geometry_msgs::TransformStamped;

constexpr int64_t kFirstStampNs = 1'000'000'000;

proto::ShadowOptions ValidOptions() {
    proto::ShadowOptions options;
    options.set_min_depth_m(0.05F);
    options.set_max_depth_m(5.0F);
    options.set_depth_scale(0.001F);
    options.set_map_frame("map");
    options.set_base_frame("base_link");
    options.set_camera_frame("camera_optical");
    options.set_map_length_x(1.0F);
    options.set_map_length_y(1.0F);
    options.set_map_resolution(0.1F);
    options.set_map_roll_threshold(0.15F);
    options.set_cell_ttl_sec(1.0F);
    options.set_max_step_height(0.20F);
    options.set_max_slope_rad(0.35F);
    options.set_obstacle_min_height(0.10F);
    options.set_robot_radius(0.05F);
    options.set_inflation_radius(0.20F);
    return options;
}

CameraInfo Camera(uint32_t width = 1, uint32_t height = 1, double fx = 10.0,
                  double fy = 10.0, double cx = 0.0, double cy = 0.0) {
    CameraInfo camera;
    camera.mutable_header()->set_frame_id("camera_optical");
    camera.set_width(width);
    camera.set_height(height);
    camera.add_k(fx);
    camera.add_k(0.0);
    camera.add_k(cx);
    camera.add_k(0.0);
    camera.add_k(fy);
    camera.add_k(cy);
    camera.add_k(0.0);
    camera.add_k(0.0);
    camera.add_k(1.0);
    return camera;
}

Image FloatDepth(uint32_t width, uint32_t height,
                 const std::vector<float>& values) {
    Image image;
    image.mutable_header()->set_frame_id("camera_optical");
    image.set_encoding("32FC1");
    image.set_width(width);
    image.set_height(height);
    image.set_is_bigendian(false);
    image.set_step(width * sizeof(float));
    std::string data(values.size() * sizeof(float), '\0');
    std::memcpy(data.data(), values.data(), data.size());
    image.set_data(data);
    return image;
}

void WriteUint16(std::string* data, std::size_t offset, uint16_t value,
                 bool big_endian) {
    for (std::size_t byte = 0; byte < sizeof(value); ++byte) {
        const std::size_t shift_byte =
            big_endian ? sizeof(value) - 1U - byte : byte;
        (*data)[offset + byte] = static_cast<char>(
            (value >> (8U * shift_byte)) & static_cast<uint16_t>(0xffU));
    }
}

void WriteFloat32(std::string* data, std::size_t offset, float value,
                  bool big_endian) {
    uint32_t bits = 0U;
    std::memcpy(&bits, &value, sizeof(bits));
    for (std::size_t byte = 0; byte < sizeof(bits); ++byte) {
        const std::size_t shift_byte =
            big_endian ? sizeof(bits) - 1U - byte : byte;
        (*data)[offset + byte] = static_cast<char>(
            (bits >> (8U * shift_byte)) & static_cast<uint32_t>(0xffU));
    }
}

Image PaddedUint16Depth(bool big_endian) {
    constexpr uint32_t kWidth = 3U;
    constexpr uint32_t kHeight = 2U;
    constexpr uint32_t kBytesPerPixel = sizeof(uint16_t);
    constexpr uint32_t kStep = kWidth * kBytesPerPixel + 3U;
    const std::vector<uint16_t> values = {1000U, 1100U, 1200U,
                                          1300U, 1400U, 1500U};
    std::string data(kHeight * kStep, static_cast<char>(0x7f));
    for (uint32_t row = 0U; row < kHeight; ++row) {
        for (uint32_t column = 0U; column < kWidth; ++column) {
            const std::size_t value_index = row * kWidth + column;
            const std::size_t offset = row * kStep + column * kBytesPerPixel;
            WriteUint16(&data, offset, values[value_index], big_endian);
        }
    }

    Image image;
    image.mutable_header()->set_frame_id("camera_optical");
    image.set_encoding("16UC1");
    image.set_width(kWidth);
    image.set_height(kHeight);
    image.set_is_bigendian(big_endian);
    image.set_step(kStep);
    image.set_data(data);
    return image;
}

Image PaddedFloatDepth(bool big_endian) {
    constexpr uint32_t kWidth = 3U;
    constexpr uint32_t kHeight = 2U;
    constexpr uint32_t kBytesPerPixel = sizeof(float);
    constexpr uint32_t kStep = kWidth * kBytesPerPixel + 5U;
    const std::vector<float> values = {1.0F, 1.1F, 1.2F, 1.3F, 1.4F, 1.5F};
    std::string data(kHeight * kStep, static_cast<char>(0x7f));
    for (uint32_t row = 0U; row < kHeight; ++row) {
        for (uint32_t column = 0U; column < kWidth; ++column) {
            const std::size_t value_index = row * kWidth + column;
            const std::size_t offset = row * kStep + column * kBytesPerPixel;
            WriteFloat32(&data, offset, values[value_index], big_endian);
        }
    }

    Image image;
    image.mutable_header()->set_frame_id("camera_optical");
    image.set_encoding("32FC1");
    image.set_width(kWidth);
    image.set_height(kHeight);
    image.set_is_bigendian(big_endian);
    image.set_step(kStep);
    image.set_data(data);
    return image;
}

Image DepthAt(float depth_m) {
    return FloatDepth(1, 1, {depth_m});
}

Image UnknownDepth() {
    return DepthAt(std::numeric_limits<float>::quiet_NaN());
}

TransformStamped CameraToMap(int64_t stamp_ns, double x = 0.0, double y = 0.0,
                             double z = 0.0) {
    TransformStamped transform;
    transform.mutable_header()->set_frame_id("map");
    transform.set_child_frame_id("camera_optical");
    transform.mutable_header()->mutable_stamp()->set_sec(
        static_cast<int32_t>(stamp_ns / 1'000'000'000));
    transform.mutable_header()->mutable_stamp()->set_nanosec(
        static_cast<uint32_t>(stamp_ns % 1'000'000'000));
    transform.mutable_transform()->mutable_translation()->set_x(x);
    transform.mutable_transform()->mutable_translation()->set_y(y);
    transform.mutable_transform()->mutable_translation()->set_z(z);
    transform.mutable_transform()->mutable_rotation()->set_w(1.0);
    return transform;
}

Odometry Odom(int64_t stamp_ns, double x = 0.0, double y = 0.0,
              double z = 1.0) {
    Odometry odometry;
    odometry.mutable_header()->set_frame_id("map");
    odometry.set_child_frame_id("base_link");
    odometry.mutable_header()->mutable_stamp()->set_sec(
        static_cast<int32_t>(stamp_ns / 1'000'000'000));
    odometry.mutable_header()->mutable_stamp()->set_nanosec(
        static_cast<uint32_t>(stamp_ns % 1'000'000'000));
    odometry.mutable_pose()->mutable_pose()->mutable_position()->set_x(x);
    odometry.mutable_pose()->mutable_pose()->mutable_position()->set_y(y);
    odometry.mutable_pose()->mutable_pose()->mutable_position()->set_z(z);
    odometry.mutable_pose()->mutable_pose()->mutable_orientation()->set_w(1.0);
    return odometry;
}

void ExpectCellUnknown(const grid_map::GridMap& map,
                       const grid_map::Position& position) {
    grid_map::Index index;
    ASSERT_TRUE(map.getIndex(position, index));
    for (const auto& layer : map.getLayers()) {
        EXPECT_FALSE(std::isfinite(map.at(layer, index))) << layer;
    }
}

void ExpectMapsEqual(const grid_map::GridMap& actual,
                     const grid_map::GridMap& expected) {
    EXPECT_EQ(actual.getLayers(), expected.getLayers());
    EXPECT_EQ(actual.getBasicLayers(), expected.getBasicLayers());
    EXPECT_EQ(actual.getTimestamp(), expected.getTimestamp());
    EXPECT_EQ(actual.getFrameId(), expected.getFrameId());
    EXPECT_TRUE(actual.getLength().isApprox(expected.getLength()));
    EXPECT_TRUE(actual.getPosition().isApprox(expected.getPosition()));
    EXPECT_TRUE((actual.getStartIndex() == expected.getStartIndex()).all());
    ASSERT_TRUE((actual.getSize() == expected.getSize()).all());
    for (const auto& layer : expected.getLayers()) {
        const auto& actual_data = actual.get(layer);
        const auto& expected_data = expected.get(layer);
        for (Eigen::Index row = 0; row < expected_data.rows(); ++row) {
            for (Eigen::Index column = 0; column < expected_data.cols();
                 ++column) {
                const float actual_value = actual_data(row, column);
                const float expected_value = expected_data(row, column);
                if (std::isnan(expected_value)) {
                    EXPECT_TRUE(std::isnan(actual_value));
                } else {
                    EXPECT_FLOAT_EQ(actual_value, expected_value);
                }
            }
        }
    }
}

void ExpectFrameRejectedAtomically(const std::string& depth_frame,
                                   const std::string& camera_frame) {
    auto options = ValidOptions();
    options.set_obstacle_min_height(1.0F);
    LocalGrid grid(options);
    ASSERT_TRUE(grid.Update(kFirstStampNs, DepthAt(1.0F), Camera(),
                            CameraToMap(kFirstStampNs), Odom(kFirstStampNs)));
    const grid_map::GridMap before = grid.map();

    constexpr int64_t kFailedStampNs = kFirstStampNs + 100'000'000;
    auto depth = DepthAt(1.2F);
    depth.mutable_header()->set_frame_id(depth_frame);
    auto camera = Camera();
    camera.mutable_header()->set_frame_id(camera_frame);
    std::string error;

    EXPECT_FALSE(grid.Update(kFailedStampNs, depth, camera,
                             CameraToMap(kFailedStampNs),
                             Odom(kFailedStampNs, 0.30), &error));
    EXPECT_FALSE(error.empty());
    ExpectMapsEqual(grid.map(), before);

    constexpr int64_t kRecoveryStampNs = kFailedStampNs + 100'000'000;
    ASSERT_TRUE(grid.Update(kRecoveryStampNs, DepthAt(1.2F), Camera(),
                            CameraToMap(kRecoveryStampNs),
                            Odom(kRecoveryStampNs)));
    grid_map::Index index;
    ASSERT_TRUE(grid.map().getIndex(grid_map::Position(0.0, 0.0), index));
    EXPECT_NEAR(grid.map().at("elevation", index), 1.1F, 1.0e-6F);
    EXPECT_NEAR(grid.map().at("variance", index), 0.01F, 1.0e-6F);
}

void ExpectOdometryShiftRejectedAtomically(double x, double y) {
    auto options = ValidOptions();
    options.set_obstacle_min_height(1.0F);
    LocalGrid grid(options);
    ASSERT_TRUE(grid.Update(kFirstStampNs, DepthAt(1.0F), Camera(),
                            CameraToMap(kFirstStampNs), Odom(kFirstStampNs)));
    const grid_map::GridMap before = grid.map();

    constexpr int64_t kFailedStampNs = kFirstStampNs + 100'000'000;
    std::string error;
    EXPECT_FALSE(grid.Update(kFailedStampNs, DepthAt(1.2F), Camera(),
                             CameraToMap(kFailedStampNs),
                             Odom(kFailedStampNs, x, y), &error));
    EXPECT_FALSE(error.empty());
    ExpectMapsEqual(grid.map(), before);

    constexpr int64_t kRecoveryStampNs = kFailedStampNs + 100'000'000;
    ASSERT_TRUE(grid.Update(kRecoveryStampNs, DepthAt(1.2F), Camera(),
                            CameraToMap(kRecoveryStampNs),
                            Odom(kRecoveryStampNs)));
    grid_map::Index index;
    ASSERT_TRUE(grid.map().getIndex(grid_map::Position(0.0, 0.0), index));
    EXPECT_NEAR(grid.map().at("elevation", index), 1.1F, 1.0e-6F);
    EXPECT_NEAR(grid.map().at("variance", index), 0.01F, 1.0e-6F);
}

void ExpectNearLimitShiftRejectedAfterRoll(double direction) {
    auto options = ValidOptions();
    options.set_obstacle_min_height(1.0F);
    LocalGrid grid(options);
    ASSERT_TRUE(grid.Update(kFirstStampNs, DepthAt(1.0F), Camera(),
                            CameraToMap(kFirstStampNs), Odom(kFirstStampNs)));

    constexpr int64_t kRollStampNs = kFirstStampNs + 100'000'000;
    ASSERT_TRUE(grid.Update(kRollStampNs, UnknownDepth(), Camera(),
                            CameraToMap(kRollStampNs),
                            Odom(kRollStampNs, 0.30)));
    ASSERT_NE(grid.map().getStartIndex().x(), 0);
    const grid_map::GridMap before = grid.map();

    using IndexScalar = grid_map::Index::Scalar;
    const double normalized_shift =
        direction *
        (static_cast<double>(std::numeric_limits<IndexScalar>::max()) - 1.0);
    const double near_limit_x =
        before.getPosition().x() + normalized_shift * before.getResolution();
    ASSERT_TRUE(std::isfinite(near_limit_x));

    constexpr int64_t kFailedStampNs = kRollStampNs + 100'000'000;
    std::string error;
    EXPECT_FALSE(grid.Update(kFailedStampNs, UnknownDepth(), Camera(),
                             CameraToMap(kFailedStampNs),
                             Odom(kFailedStampNs, near_limit_x), &error));
    EXPECT_FALSE(error.empty());
    ExpectMapsEqual(grid.map(), before);

    constexpr int64_t kRecoveryStampNs = kFailedStampNs + 100'000'000;
    ASSERT_TRUE(grid.Update(kRecoveryStampNs, DepthAt(1.2F), Camera(),
                            CameraToMap(kRecoveryStampNs),
                            Odom(kRecoveryStampNs, 0.30)));
    grid_map::Index index;
    ASSERT_TRUE(grid.map().getIndex(grid_map::Position(0.0, 0.0), index));
    EXPECT_NEAR(grid.map().at("elevation", index), 1.1F, 1.0e-6F);
    EXPECT_NEAR(grid.map().at("variance", index), 0.01F, 1.0e-6F);
}

TEST(LocalGridTest, InitializesExactlyFourUnknownLayers) {
    LocalGrid grid(ValidOptions());
    const auto& map = grid.map();
    const std::vector<std::string> expected_layers = {
        "elevation", "variance", "obstacle", "traversability"};

    EXPECT_EQ(map.getLayers(), expected_layers);
    EXPECT_EQ(map.getBasicLayers(), expected_layers);
    EXPECT_EQ(map.getFrameId(), "map");
    EXPECT_TRUE((map.getSize() == grid_map::Size(10, 10)).all());
    EXPECT_FALSE(
        std::isfinite(map.at("traversability", grid_map::Index(0, 0))));
    for (const auto& layer : map.getLayers()) {
        const auto& data = map.get(layer);
        for (Eigen::Index row = 0; row < data.rows(); ++row) {
            for (Eigen::Index column = 0; column < data.cols(); ++column) {
                EXPECT_TRUE(std::isnan(data(row, column))) << layer;
            }
        }
    }
}

TEST(LocalGridTest, AccumulatesPerCellElevationMeanAndVariance) {
    auto options = ValidOptions();
    options.set_obstacle_min_height(1.0F);
    LocalGrid grid(options);

    ASSERT_TRUE(grid.Update(kFirstStampNs, DepthAt(1.0F), Camera(),
                            CameraToMap(kFirstStampNs), Odom(kFirstStampNs)));
    ASSERT_TRUE(grid.Update(kFirstStampNs + 100'000'000, DepthAt(1.2F),
                            Camera(), CameraToMap(kFirstStampNs + 100'000'000),
                            Odom(kFirstStampNs + 100'000'000)));

    grid_map::Index index;
    ASSERT_TRUE(grid.map().getIndex(grid_map::Position(0.0, 0.0), index));
    EXPECT_NEAR(grid.map().at("elevation", index), 1.1F, 1.0e-6F);
    EXPECT_NEAR(grid.map().at("variance", index), 0.01F, 1.0e-6F);
    EXPECT_FLOAT_EQ(grid.map().at("obstacle", index), 0.0F);
    EXPECT_GE(grid.map().at("traversability", index), 0.0F);
    EXPECT_LE(grid.map().at("traversability", index), 1.0F);
}

TEST(LocalGridTest, DecodesPaddedDepthRowsInBothEndianModes) {
    auto options = ValidOptions();
    options.set_obstacle_min_height(1.0F);
    const auto camera = Camera(3U, 2U, 1.0e6, 1.0e6);

    for (const bool big_endian : {false, true}) {
        for (const auto& depth :
             {PaddedUint16Depth(big_endian), PaddedFloatDepth(big_endian)}) {
            SCOPED_TRACE(::testing::Message() << "encoding=" << depth.encoding()
                                              << ", big_endian=" << big_endian);
            const uint32_t bytes_per_pixel =
                depth.encoding() == "16UC1" ? sizeof(uint16_t) : sizeof(float);
            ASSERT_GT(depth.step(), depth.width() * bytes_per_pixel);
            LocalGrid grid(options);
            ASSERT_TRUE(grid.Update(kFirstStampNs, depth, camera,
                                    CameraToMap(kFirstStampNs, 0.025, 0.025),
                                    Odom(kFirstStampNs)));

            grid_map::Index index;
            ASSERT_TRUE(
                grid.map().getIndex(grid_map::Position(0.025, 0.025), index));
            EXPECT_NEAR(grid.map().at("elevation", index), 1.25F, 1.0e-6F);
            EXPECT_NEAR(grid.map().at("variance", index), 0.02916667F, 1.0e-6F);
        }
    }
}

TEST(LocalGridTest, RejectsEmptyDepthOrCameraFrameAtomically) {
    {
        SCOPED_TRACE("empty depth frame");
        ExpectFrameRejectedAtomically("", "camera_optical");
    }
    {
        SCOPED_TRACE("empty camera frame");
        ExpectFrameRejectedAtomically("camera_optical", "");
    }
}

TEST(LocalGridTest, RejectsMismatchedDepthOrCameraFrameAtomically) {
    {
        SCOPED_TRACE("mismatched depth frame");
        ExpectFrameRejectedAtomically("other_camera", "camera_optical");
    }
    {
        SCOPED_TRACE("mismatched camera frame");
        ExpectFrameRejectedAtomically("camera_optical", "other_camera");
    }
}

TEST(LocalGridTest, ClassifiesHeightAboveSupportPlaneAsObstacle) {
    LocalGrid grid(ValidOptions());

    ASSERT_TRUE(grid.Update(kFirstStampNs, DepthAt(1.2F), Camera(),
                            CameraToMap(kFirstStampNs), Odom(kFirstStampNs)));

    grid_map::Index index;
    ASSERT_TRUE(grid.map().getIndex(grid_map::Position(0.0, 0.0), index));
    EXPECT_FLOAT_EQ(grid.map().at("obstacle", index), 1.0F);
    EXPECT_FLOAT_EQ(grid.map().at("traversability", index), 1.0F);
}

TEST(LocalGridTest, ExcessiveNeighborSlopeIsImpassable) {
    auto options = ValidOptions();
    options.set_obstacle_min_height(1.0F);
    options.set_max_step_height(0.5F);
    options.set_max_slope_rad(0.5F);
    LocalGrid grid(options);
    const auto depth = FloatDepth(2, 1, {1.0F, 1.15F});
    const auto camera = Camera(2, 1, 20.0, 10.0, 0.0, 0.0);

    ASSERT_TRUE(grid.Update(kFirstStampNs, depth, camera,
                            CameraToMap(kFirstStampNs), Odom(kFirstStampNs)));

    grid_map::Index raised_index;
    ASSERT_TRUE(
        grid.map().getIndex(grid_map::Position(0.0575, 0.0), raised_index));
    EXPECT_FLOAT_EQ(grid.map().at("obstacle", raised_index), 0.0F);
    EXPECT_FLOAT_EQ(grid.map().at("traversability", raised_index), 1.0F);
}

TEST(LocalGridTest, ModerateNeighborSlopeProducesNormalizedCost) {
    auto options = ValidOptions();
    options.set_obstacle_min_height(1.0F);
    options.set_max_step_height(0.5F);
    options.set_max_slope_rad(1.0F);
    LocalGrid grid(options);
    const auto depth = FloatDepth(2, 1, {1.0F, 1.05F});
    const auto camera = Camera(2, 1, 20.0, 10.0, 0.0, 0.0);

    ASSERT_TRUE(grid.Update(kFirstStampNs, depth, camera,
                            CameraToMap(kFirstStampNs), Odom(kFirstStampNs)));

    grid_map::Index raised_index;
    ASSERT_TRUE(
        grid.map().getIndex(grid_map::Position(0.0525, 0.0), raised_index));
    EXPECT_FLOAT_EQ(grid.map().at("obstacle", raised_index), 0.0F);
    EXPECT_GT(grid.map().at("traversability", raised_index), 0.0F);
    EXPECT_LT(grid.map().at("traversability", raised_index), 1.0F);
}

TEST(LocalGridTest, InflationRaisesNearbyObservedTraversability) {
    auto options = ValidOptions();
    options.set_max_step_height(10.0F);
    options.set_max_slope_rad(10.0F);
    LocalGrid grid(options);
    const auto depth = FloatDepth(5, 1, {1.0F, 1.0F, 1.11F, 1.0F, 1.0F});
    const auto camera = Camera(5, 1, 20.0, 10.0, 2.0, 0.0);

    ASSERT_TRUE(grid.Update(kFirstStampNs, depth, camera,
                            CameraToMap(kFirstStampNs), Odom(kFirstStampNs)));

    grid_map::Index nearby_index;
    ASSERT_TRUE(
        grid.map().getIndex(grid_map::Position(0.05, 0.0), nearby_index));
    EXPECT_FLOAT_EQ(grid.map().at("obstacle", nearby_index), 0.0F);
    EXPECT_GT(grid.map().at("traversability", nearby_index), 0.5F);
    EXPECT_LE(grid.map().at("traversability", nearby_index), 1.0F);
}

TEST(LocalGridTest, RobotRadiusMakesNearbyObservedCellImpassable) {
    auto options = ValidOptions();
    options.set_max_step_height(10.0F);
    options.set_max_slope_rad(10.0F);
    options.set_robot_radius(0.15F);
    LocalGrid grid(options);
    const auto depth = FloatDepth(5, 1, {1.0F, 1.0F, 1.11F, 1.0F, 1.0F});
    const auto camera = Camera(5, 1, 20.0, 10.0, 2.0, 0.0);

    ASSERT_TRUE(grid.Update(kFirstStampNs, depth, camera,
                            CameraToMap(kFirstStampNs), Odom(kFirstStampNs)));

    grid_map::Index nearby_index;
    ASSERT_TRUE(
        grid.map().getIndex(grid_map::Position(0.05, 0.0), nearby_index));
    EXPECT_FLOAT_EQ(grid.map().at("obstacle", nearby_index), 0.0F);
    EXPECT_FLOAT_EQ(grid.map().at("traversability", nearby_index), 1.0F);
}

TEST(LocalGridTest, RollingPreservesOverlapAndLeavesNewRegionUnknown) {
    LocalGrid grid(ValidOptions());
    ASSERT_TRUE(grid.Update(kFirstStampNs, DepthAt(1.0F), Camera(),
                            CameraToMap(kFirstStampNs), Odom(kFirstStampNs)));

    constexpr int64_t kSecondStampNs = kFirstStampNs + 100'000'000;
    ASSERT_TRUE(grid.Update(kSecondStampNs, UnknownDepth(), Camera(),
                            CameraToMap(kSecondStampNs),
                            Odom(kSecondStampNs, 0.30)));

    grid_map::Index preserved_index;
    ASSERT_TRUE(
        grid.map().getIndex(grid_map::Position(0.0, 0.0), preserved_index));
    EXPECT_FLOAT_EQ(grid.map().at("elevation", preserved_index), 1.0F);
    EXPECT_FLOAT_EQ(grid.map().at("variance", preserved_index), 0.0F);
    EXPECT_FLOAT_EQ(grid.map().at("obstacle", preserved_index), 0.0F);
    EXPECT_FLOAT_EQ(grid.map().at("traversability", preserved_index), 0.0F);
    EXPECT_NEAR(grid.map().getPosition().x(), 0.30, 1.0e-6);
    ExpectCellUnknown(grid.map(), grid_map::Position(0.75, 0.0));
}

TEST(LocalGridTest, RejectsUnrepresentableOdometryShiftAtomically) {
    const double huge = std::numeric_limits<double>::max();
    {
        SCOPED_TRACE("unrepresentable x shift");
        ExpectOdometryShiftRejectedAtomically(huge, 0.0);
    }
    {
        SCOPED_TRACE("unrepresentable y shift");
        ExpectOdometryShiftRejectedAtomically(0.0, huge);
    }
}

TEST(LocalGridTest, RejectsPositiveAndNegativeNearLimitShiftsAfterRoll) {
    {
        SCOPED_TRACE("positive near-limit shift");
        ExpectNearLimitShiftRejectedAfterRoll(1.0);
    }
    {
        SCOPED_TRACE("negative near-limit shift");
        ExpectNearLimitShiftRejectedAfterRoll(-1.0);
    }
}

TEST(LocalGridTest, ExpiredCellReturnsToUnknownInEveryLayer) {
    LocalGrid grid(ValidOptions());
    ASSERT_TRUE(grid.Update(kFirstStampNs, DepthAt(1.0F), Camera(),
                            CameraToMap(kFirstStampNs), Odom(kFirstStampNs)));

    constexpr int64_t kExpiredStampNs = kFirstStampNs + 1'100'000'000;
    ASSERT_TRUE(grid.Update(kExpiredStampNs, UnknownDepth(), Camera(),
                            CameraToMap(kExpiredStampNs),
                            Odom(kExpiredStampNs)));

    ExpectCellUnknown(grid.map(), grid_map::Position(0.0, 0.0));
}

TEST(LocalGridTest, FailedUpdateLeavesAllMapStateUnchanged) {
    auto options = ValidOptions();
    options.set_obstacle_min_height(1.0F);
    LocalGrid grid(options);
    ASSERT_TRUE(grid.Update(kFirstStampNs, DepthAt(1.0F), Camera(),
                            CameraToMap(kFirstStampNs), Odom(kFirstStampNs)));
    const grid_map::GridMap before = grid.map();
    constexpr int64_t kFailedStampNs = kFirstStampNs + 100'000'000;
    const auto invalid_transform = CameraToMap(
        kFailedStampNs, 0.0, 0.0, std::numeric_limits<double>::max());
    std::string error;

    EXPECT_FALSE(grid.Update(kFailedStampNs, DepthAt(1.0F), Camera(),
                             invalid_transform, Odom(kFailedStampNs, 0.30),
                             &error));
    EXPECT_FALSE(error.empty());
    ExpectMapsEqual(grid.map(), before);

    constexpr int64_t kRecoveryStampNs = kFailedStampNs + 100'000'000;
    ASSERT_TRUE(grid.Update(kRecoveryStampNs, DepthAt(1.2F), Camera(),
                            CameraToMap(kRecoveryStampNs),
                            Odom(kRecoveryStampNs)));
    grid_map::Index index;
    ASSERT_TRUE(grid.map().getIndex(grid_map::Position(0.0, 0.0), index));
    EXPECT_NEAR(grid.map().at("elevation", index), 1.1F, 1.0e-6F);
    EXPECT_NEAR(grid.map().at("variance", index), 0.01F, 1.0e-6F);
}

TEST(LocalGridTest, ConvertsWithExactMapFrameTimestampAndLayers) {
    LocalGrid grid(ValidOptions());
    constexpr int64_t kStampNs = 2'500'000'000;
    ASSERT_TRUE(grid.Update(kStampNs, DepthAt(1.0F), Camera(),
                            CameraToMap(kStampNs), Odom(kStampNs)));
    GridMapMessage message;
    message.add_layers("stale");
    std::string error = "stale error";

    ASSERT_TRUE(grid.ToMessage(&message, &error)) << error;
    EXPECT_TRUE(error.empty());
    EXPECT_EQ(message.info().header().frame_id(), "map");
    EXPECT_EQ(message.info().header().stamp().sec(), 2);
    EXPECT_EQ(message.info().header().stamp().nanosec(), 500'000'000U);
    ASSERT_EQ(message.layers_size(), 4);
    EXPECT_EQ(message.layers(0), "elevation");
    EXPECT_EQ(message.layers(1), "variance");
    EXPECT_EQ(message.layers(2), "obstacle");
    EXPECT_EQ(message.layers(3), "traversability");
    EXPECT_EQ(message.data_size(), 4);
}

TEST(LocalGridTest, ClearRestoresInitialUnknownMap) {
    LocalGrid grid(ValidOptions());
    ASSERT_TRUE(grid.Update(kFirstStampNs, DepthAt(1.0F), Camera(),
                            CameraToMap(kFirstStampNs), Odom(kFirstStampNs)));

    grid.Clear();

    EXPECT_EQ(grid.map().getTimestamp(), 0U);
    EXPECT_TRUE(grid.map().getPosition().isZero());
    ExpectCellUnknown(grid.map(), grid_map::Position(0.0, 0.0));
}

}  // namespace
}  // namespace shadow
}  // namespace perception
}  // namespace autonomy
