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

#include <atomic>
#include <gtest/gtest.h>

#include "autodriver/sensor_hub.hpp"
#include "autodriver/types/sensor_sample.hpp"
#include "autodriver/types/sensor_type.hpp"
#include "autolink/time/duration.hpp"
#include "autolink/time/time.hpp"

TEST(SensorHub, AlignsPushedImu) {
    autodriver::SensorHub::Options options;
    options.publish_period = autolink::Duration(10'000'000);
    options.alignment_window = autolink::Duration(200'000'000);

    autodriver::SensorHub hub(options);
    std::atomic<int> n{0};
    hub.SetAlignedCallback([&](const autodriver::AlignedSnapshot& snapshot) {
        ++n;
        EXPECT_NE(snapshot.Get<autodriver::ImuSample>("imu/t"), nullptr);
    });
    ASSERT_TRUE(hub.Start());
    hub.PushSample(std::make_unique<autodriver::ImuSample>(
        "imu/t", autolink::Time::Now(), automsgs::msgs::sensor_msgs::Imu{}));
    autolink::Duration(80'000'000).Sleep();
    hub.Stop();
    EXPECT_GT(n.load(), 0);
}

TEST(SensorHub, AlignsNImusBySensorId) {
    autodriver::SensorHub::Options options;
    options.publish_period = autolink::Duration(10'000'000);
    options.alignment_window = autolink::Duration(200'000'000);

    autodriver::SensorHub hub(options);
    std::atomic<int> n{0};
    hub.SetAlignedCallback([&](const autodriver::AlignedSnapshot& snapshot) {
        ++n;
        EXPECT_NE(snapshot.Get<autodriver::ImuSample>("imu/a"), nullptr);
        EXPECT_NE(snapshot.Get<autodriver::ImuSample>("imu/b"), nullptr);
        EXPECT_EQ(
            snapshot.GetAll<autodriver::ImuSample>(autodriver::SensorType::kImu)
                .size(),
            2u);
    });
    ASSERT_TRUE(hub.Start());
    const autolink::Time now = autolink::Time::Now();
    hub.PushSample(std::make_unique<autodriver::ImuSample>(
        "imu/a", now, automsgs::msgs::sensor_msgs::Imu{}));
    hub.PushSample(std::make_unique<autodriver::ImuSample>(
        "imu/b", now, automsgs::msgs::sensor_msgs::Imu{}));
    autolink::Duration(80'000'000).Sleep();
    hub.Stop();
    EXPECT_GT(n.load(), 0);
}
