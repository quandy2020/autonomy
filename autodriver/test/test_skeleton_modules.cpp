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

#include <cstring>

#include "autodriver/camera/backend_registry.hpp"
#include "autodriver/gps/parser/parser.hpp"
#include "autodriver/lidar/packet_queue.hpp"
#include "autodriver/microphone/backend_registry.hpp"
#include "autodriver/radar/backend_registry.hpp"

TEST(SkeletonModules, RadarMicrophoneSmartereyeRegistered) {
    EXPECT_TRUE(autodriver::radar::RadarBackendRegistry::Instance().Has("conti"));
    EXPECT_TRUE(
        autodriver::microphone::MicrophoneBackendRegistry::Instance().Has(
            "respeaker"));
    EXPECT_TRUE(
        autodriver::camera::CameraBackendRegistry::Instance().Has("smartereye"));

    autodriver::hardware::DriverParams params;
    EXPECT_EQ(autodriver::radar::RadarBackendRegistry::Instance().Create(
                  "conti", "radar/front", params),
              nullptr);
    EXPECT_EQ(
        autodriver::microphone::MicrophoneBackendRegistry::Instance().Create(
            "respeaker", "mic/0", params),
        nullptr);
    EXPECT_EQ(autodriver::camera::CameraBackendRegistry::Instance().Create(
                  "smartereye", "camera/se", params),
              nullptr);
}

TEST(GnssParser, NmeaRegisteredAndParsesGga) {
    EXPECT_TRUE(autodriver::gps::GnssParserRegistry::Instance().Has("nmea"));
    auto parser = autodriver::gps::GnssParserRegistry::Instance().Create("nmea");
    ASSERT_NE(parser, nullptr);
    const char* line =
        "$GNGGA,123519,4807.038,N,01131.000,E,1,08,0.9,545.4,M,46.9,M,,*59\n";
    auto fix = parser->Consume(reinterpret_cast<const std::uint8_t*>(line),
                               std::strlen(line));
    ASSERT_TRUE(fix.has_value());
    EXPECT_NEAR(fix->latitude_deg, 48.1173, 0.001);
}

TEST(PacketQueue, DropsOldestWhenFull) {
    autodriver::lidar::PacketQueue<int> q(2);
    q.Push(1);
    q.Push(2);
    q.Push(3);
    EXPECT_EQ(q.dropped(), 1u);
    EXPECT_EQ(q.TryPop().value(), 2);
    EXPECT_EQ(q.TryPop().value(), 3);
    EXPECT_FALSE(q.TryPop().has_value());
}
