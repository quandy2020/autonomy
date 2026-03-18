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

#include <automsgs/msgs/geometry_msgs/vector3.pb.h>

TEST(Vector3, SerializationRoundTrip)
{
  automsgs::msgs::geometry_msgs::Vector3 msg;
  msg.set_x(1.0);
  msg.set_y(2.0);
  msg.set_z(3.0);

  std::string serialized;
  msg.SerializeToString(&serialized);
  EXPECT_GT(serialized.size(), 0u);

  automsgs::msgs::geometry_msgs::Vector3 decoded;
  EXPECT_TRUE(decoded.ParseFromString(serialized));
  EXPECT_DOUBLE_EQ(decoded.x(), 1.0);
  EXPECT_DOUBLE_EQ(decoded.y(), 2.0);
  EXPECT_DOUBLE_EQ(decoded.z(), 3.0);
}

TEST(Vector3, DefaultValues)
{
  automsgs::msgs::geometry_msgs::Vector3 msg;
  EXPECT_DOUBLE_EQ(msg.x(), 0.0);
  EXPECT_DOUBLE_EQ(msg.y(), 0.0);
  EXPECT_DOUBLE_EQ(msg.z(), 0.0);
}
