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

#include <array>
#include <cstdint>
#include <vector>

#include <gtest/gtest.h>

#include "autodriver/hardware/wit_motion_parser.hpp"

using autodriver::protocol::ParseNmea2000LatLonFrame;
using autodriver::protocol::WitMotionParser;

namespace {

std::uint8_t WitChecksum(const std::vector<std::uint8_t> & frame)
{
  std::uint8_t sum = 0;
  for (std::size_t i = 0; i < 10; ++i) {
    sum += frame[i];
  }
  return sum;
}

}  // namespace

TEST(WitMotion, ParsesAccelAndGyroPackets)
{
  WitMotionParser parser(1.0, 0.01);

  std::vector<std::uint8_t> accel = {
    0x55, 0x51, 0x10, 0x00, 0x20, 0x00, 0x30, 0x00, 0x00, 0x00, 0x00};
  accel[10] = WitChecksum(accel);

  for (std::uint8_t byte : accel) {
    parser.Feed(byte);
  }
  EXPECT_TRUE(parser.state().have_accel);
  EXPECT_FALSE(parser.HasCompleteSample());

  std::vector<std::uint8_t> gyro = {
    0x55, 0x52, 0x01, 0x00, 0x02, 0x00, 0x03, 0x00, 0x00, 0x00, 0x00};
  gyro[10] = WitChecksum(gyro);

  for (std::uint8_t byte : gyro) {
    parser.Feed(byte);
  }

  EXPECT_TRUE(parser.HasCompleteSample());
  EXPECT_NEAR(parser.state().linear_acceleration[0], 16.0, 1e-6);
  EXPECT_NEAR(parser.state().angular_velocity[2], 3.0 * 0.01, 1e-6);
}

TEST(WitMotion, ParsesNmea2000LatLon)
{
  std::array<std::uint8_t, 8> data{};
  data[0] = 0x80;
  data[1] = 0x96;
  data[2] = 0x98;
  data[3] = 0x00;
  data[4] = 0x00;
  data[5] = 0x2D;
  data[6] = 0x31;
  data[7] = 0x01;

  const auto fix = ParseNmea2000LatLonFrame(data.data(), data.size());
  ASSERT_TRUE(fix.has_value());
  EXPECT_NEAR(fix->latitude_deg, 1.0, 1e-6);
  EXPECT_NEAR(fix->longitude_deg, 2.0, 1e-6);
}
