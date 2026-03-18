/*
 * Copyright 2025 The Openbot Authors (duyongquan)
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

#include <automsgs/msgs/geometry_msgs/pose.pb.h>
#include <automsgs/msgs/geometry_msgs/point.pb.h>
#include <automsgs/msgs/geometry_msgs/quaternion.pb.h>
#include <automsgs/msgs/geometry_msgs/vector3.pb.h>
#include <automsgs/msgs/std_msgs/header.pb.h>
#include <automsgs/msgs/std_msgs/string.pb.h>

TEST(Messages, NestedHeaders)
{
  automsgs::msgs::geometry_msgs::Pose pose;
  std::string pose_serialized;
  pose.SerializeToString(&pose_serialized);
  EXPECT_EQ(0u, pose_serialized.size());

  automsgs::msgs::std_msgs::Header header;
  std::string header_serialized;
  header.SerializeToString(&header_serialized);
  EXPECT_EQ(0u, header_serialized.size());

  header.mutable_stamp()->set_sec(123);
  header.mutable_stamp()->set_nanosec(456);
  header.SerializeToString(&header_serialized);
  EXPECT_GT(header_serialized.size(), 0u);

  pose.mutable_position()->set_x(1.0);
  pose.mutable_position()->set_y(2.0);
  pose.mutable_position()->set_z(3.0);
  pose.mutable_orientation()->set_w(1.0);
  pose.SerializeToString(&pose_serialized);
  EXPECT_GT(pose_serialized.size(), 0u);

  automsgs::msgs::std_msgs::String str_msg;
  str_msg.set_data("hello");
  std::string str_serialized;
  str_msg.SerializeToString(&str_serialized);
  EXPECT_GT(str_serialized.size(), 0u);

  automsgs::msgs::std_msgs::String str_decoded;
  EXPECT_TRUE(str_decoded.ParseFromString(str_serialized));
  EXPECT_EQ(str_decoded.data(), "hello");
}
