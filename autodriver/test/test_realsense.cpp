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

#include "autodriver/hardware/realsense_device_hub.hpp"

TEST(RealSenseTypes, ParsesStreamNames)
{
  using autodriver::hardware::realsense::ParseStreamKind;
  using autodriver::hardware::realsense::StreamKind;

  EXPECT_EQ(ParseStreamKind("", StreamKind::kColor), StreamKind::kColor);
  EXPECT_EQ(ParseStreamKind("depth", StreamKind::kColor), StreamKind::kDepth);
  EXPECT_EQ(ParseStreamKind("ir", StreamKind::kColor), StreamKind::kInfrared1);
  EXPECT_EQ(ParseStreamKind("ir2", StreamKind::kColor), StreamKind::kInfrared2);
  EXPECT_EQ(ParseStreamKind("aligned_depth_to_color", StreamKind::kColor),
            StreamKind::kAlignedDepthToColor);
  EXPECT_EQ(ParseStreamKind("pointcloud", StreamKind::kColor),
            StreamKind::kPointCloud);
}

TEST(RealSenseTypes, MatchesModelFilter)
{
  using autodriver::hardware::realsense::MatchesModelFilter;

  EXPECT_TRUE(MatchesModelFilter("Intel RealSense D435I", "D435"));
  EXPECT_TRUE(MatchesModelFilter("Intel RealSense D455", "d455"));
  EXPECT_FALSE(MatchesModelFilter("Intel RealSense D455", "D435"));
  EXPECT_TRUE(MatchesModelFilter("Intel RealSense D455", ""));
}

TEST(RealSenseTypes, EncodingForStreams)
{
  using autodriver::hardware::realsense::EncodingForStreamKind;
  using autodriver::hardware::realsense::StreamKind;

  EXPECT_EQ(EncodingForStreamKind(StreamKind::kColor), "rgb8");
  EXPECT_EQ(EncodingForStreamKind(StreamKind::kDepth), "16UC1");
  EXPECT_EQ(EncodingForStreamKind(StreamKind::kInfrared1), "mono8");
}
