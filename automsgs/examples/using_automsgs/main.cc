// Copyright 2025 The Openbot Authors (duyongquan)
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//      http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

// Example that uses automsgs message types (no Factory; direct include).
// Build and run: ./using_automsgs

#include <automsgs/msgs/geometry_msgs/pose.pb.h>
#include <automsgs/msgs/geometry_msgs/point.pb.h>
#include <automsgs/msgs/geometry_msgs/quaternion.pb.h>
#include <automsgs/msgs/std_msgs/string.pb.h>

#include <google/protobuf/text_format.h>

#include <iostream>

int main()
{
  automsgs::msgs::geometry_msgs::Pose pose;
  pose.mutable_position()->set_x(1.0);
  pose.mutable_position()->set_y(2.0);
  pose.mutable_position()->set_z(3.0);
  pose.mutable_orientation()->set_w(1.0);
  pose.mutable_orientation()->set_x(0.0);
  pose.mutable_orientation()->set_y(0.0);
  pose.mutable_orientation()->set_z(0.0);

  std::cout << "Pose message:\n" << pose.DebugString() << std::endl;

  automsgs::msgs::std_msgs::String str_msg;
  str_msg.set_data("Hello from using_automsgs");
  std::cout << "String message: " << str_msg.data() << std::endl;

  return 0;
}
