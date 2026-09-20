/*
 * Copyright 2026 Autodriver contributors duyongquan (quandy2020@126.com)
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include "autodriver/joy/dualsense_profile.hpp"
#include "autodriver/joy/joy_mapper.hpp"

#include <gtest/gtest.h>

#include <vector>

using autodriver::joy::ApplyDeadzone;
using autodriver::joy::MapDifferential;

TEST(JoyMapper, DeadzoneZerosInside) {
  EXPECT_FLOAT_EQ(ApplyDeadzone(0.0f, 0.1f), 0.0f);
  EXPECT_FLOAT_EQ(ApplyDeadzone(0.05f, 0.1f), 0.0f);
  EXPECT_FLOAT_EQ(ApplyDeadzone(-0.05f, 0.1f), 0.0f);
}

TEST(JoyMapper, DeadzoneRescalesOutside) {
  EXPECT_NEAR(ApplyDeadzone(1.0f, 0.1f), 1.0f, 1e-5);
  EXPECT_NEAR(ApplyDeadzone(-1.0f, 0.1f), -1.0f, 1e-5);
  EXPECT_GT(ApplyDeadzone(0.55f, 0.1f), 0.0f);
}

TEST(JoyMapper, RequireEnableBlocksMotion) {
  const std::vector<float> axes{0.0f, -1.0f};
  const std::vector<int32_t> buttons{0, 0, 0, 0, 0};
  const auto twist =
      MapDifferential(axes, buttons, 1, 0, true, false, 0.05f, 0.5, 1.0, true,
                      4);
  EXPECT_DOUBLE_EQ(twist.linear_x, 0.0);
  EXPECT_DOUBLE_EQ(twist.angular_z, 0.0);
}

TEST(JoyMapper, EnableAllowsForward) {
  const std::vector<float> axes{0.0f, -1.0f};
  const std::vector<int32_t> buttons{0, 0, 0, 0, 1};
  const auto twist =
      MapDifferential(axes, buttons, 1, 0, true, false, 0.05f, 0.5, 1.0, true,
                      4);
  EXPECT_NEAR(twist.linear_x, 0.5, 1e-6);
  EXPECT_DOUBLE_EQ(twist.angular_z, 0.0);
}

TEST(DualSenseProfile, AppliesLeftStickArcadeAndL1) {
  autodriver::Config::Joy options;
  options.profile = "dualsense";
  options.frame_id = "keep_me_if_not_applied";
  ASSERT_TRUE(autodriver::joy::ApplyJoyProfile("dualsense", &options));
  EXPECT_EQ(options.linear_axis, 1);
  EXPECT_EQ(options.angular_axis, 0);
  EXPECT_TRUE(options.invert_linear);
  EXPECT_EQ(options.enable_button, 4);
  EXPECT_TRUE(options.require_enable);
  EXPECT_EQ(options.frame_id, "dualsense");
  EXPECT_NEAR(options.deadzone, 0.08f, 1e-5);
}

TEST(DualSenseProfile, Ps5Alias) {
  autodriver::Config::Joy options;
  EXPECT_TRUE(autodriver::joy::ApplyJoyProfile("ps5", &options));
  EXPECT_EQ(options.enable_button, 4);
}
