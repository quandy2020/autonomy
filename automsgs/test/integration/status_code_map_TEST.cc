/*
 * Copyright 2026 The Openbot Authors (duyongquan)
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

#include <automsgs/msgs/status_msgs/status_code_map.hpp>

namespace sm = automsgs::msgs::status_msgs;

TEST(StatusCodeMap, ThinPassthrough) {
  EXPECT_TRUE(sm::IsThinStatusCode(sm::OK));
  EXPECT_TRUE(sm::IsThinStatusCode(sm::NAVIGATION_BUSY));
  EXPECT_TRUE(sm::IsThinStatusCode(sm::TELEOP_COLLISION));
  EXPECT_TRUE(sm::IsThinStatusCode(sm::SYSTEM_ESTOP));
  EXPECT_FALSE(sm::IsThinStatusCode(sm::CONTROL_COLLISION));
  EXPECT_FALSE(sm::IsThinStatusCode(sm::PLANNING_NO_PATH_FOUND));

  EXPECT_EQ(sm::ToThinStatusCode(sm::OK), sm::OK);
  EXPECT_EQ(sm::ToThinStatusCode(sm::NAVIGATION_NO_PATH), sm::NAVIGATION_NO_PATH);
  EXPECT_EQ(sm::ToThinStatusCode(sm::MAPPING_BUSY), sm::MAPPING_BUSY);
}

TEST(StatusCodeMap, FineToThinSamples) {
  EXPECT_EQ(sm::ToThinStatusCode(sm::CONTROL_CANCELLED), sm::NAVIGATION_CANCELLED);
  EXPECT_EQ(sm::ToThinStatusCode(sm::CONTROL_COLLISION), sm::NAVIGATION_PATH_BLOCKED);
  EXPECT_EQ(sm::ToThinStatusCode(sm::CONTROL_ESTOP_ERROR), sm::SYSTEM_ESTOP);
  EXPECT_EQ(sm::ToThinStatusCode(sm::LOCALIZATION_NOT_READY),
            sm::LOCALIZATION_UNAVAILABLE);
  EXPECT_EQ(sm::ToThinStatusCode(sm::PLANNING_NO_PATH_FOUND), sm::NAVIGATION_NO_PATH);
  EXPECT_EQ(sm::ToThinStatusCode(sm::PLANNING_TIMEOUT), sm::NAVIGATION_TIMEOUT);
  EXPECT_EQ(sm::ToThinStatusCode(sm::SMOOTHER_PATH_IN_COLLISION),
            sm::NAVIGATION_NO_PATH);
  EXPECT_EQ(sm::ToThinStatusCode(sm::SPIN_COLLISION_AHEAD), sm::TELEOP_COLLISION);
  EXPECT_EQ(sm::ToThinStatusCode(sm::TASK_BT_CANCELLED), sm::TASK_CANCELLED);
  EXPECT_EQ(sm::ToThinStatusCode(sm::TASK_TIMEOUT), sm::DEADLINE_EXCEEDED);
  EXPECT_EQ(sm::ToThinStatusCode(sm::MAP_OUT_OF_MAP), sm::MAP_INVALID);
  EXPECT_TRUE(sm::IsThinStatusCode(sm::ToThinStatusCode(sm::CONTROL_UNKNOWN)));
}
