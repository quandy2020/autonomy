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

#include "autodriver/protocol/nmea0183.hpp"

using autodriver::GpsFixStatus;
using autodriver::protocol::ParseGgaSentence;
using autodriver::protocol::ParseRmcSentence;

TEST(Nmea0183, ParseGgaFix)
{
  const std::string sentence =
    "$GNGGA,123519,4807.038,N,01131.000,E,1,08,0.9,545.4,M,46.9,M,,*59";
  const auto fix = ParseGgaSentence(sentence);
  ASSERT_TRUE(fix.has_value());
  EXPECT_NEAR(fix->latitude_deg, 48.1173, 0.001);
  EXPECT_NEAR(fix->longitude_deg, 11.5167, 0.001);
  EXPECT_NEAR(fix->altitude_m, 545.4, 0.1);
  EXPECT_EQ(fix->fix_status, GpsFixStatus::kFix3D);
}

TEST(Nmea0183, ParseRmcFix)
{
  const std::string sentence =
    "$GNRMC,123519,A,4807.038,N,01131.000,E,022.4,084.4,230394,003.1,W*74";
  const auto fix = ParseRmcSentence(sentence);
  ASSERT_TRUE(fix.has_value());
  EXPECT_NEAR(fix->latitude_deg, 48.1173, 0.001);
  EXPECT_NEAR(fix->longitude_deg, 11.5167, 0.001);
  EXPECT_EQ(fix->fix_status, GpsFixStatus::kFix2D);
}

TEST(Nmea0183, RejectsBadChecksum)
{
  const std::string sentence =
    "$GNGGA,123519,4807.038,N,01131.000,E,1,08,0.9,545.4,M,46.9,M,,*00";
  EXPECT_FALSE(ParseGgaSentence(sentence).has_value());
}
