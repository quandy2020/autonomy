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

#include <automsgs/custom_msgs/foo.pb.h>
#include <automsgs/custom_msgs/bar.pb.h>
#include <automsgs/custom_msgs/baz.pb.h>

#include <iostream>

// Example that uses custom message types generated from proto.
// Run: ./generating_custom_msgs

int main()
{
  automsgs::custom_msgs::BazStamped msg;

  auto* header = msg.mutable_header();
  auto* baz = msg.mutable_baz();

  header->mutable_stamp()->set_sec(100);
  header->mutable_stamp()->set_nanosec(100);
  header->set_frame_id("automsgs_custom_msgs");

  baz->mutable_foo()->set_value(1.0);
  baz->mutable_bar()->set_value(2.0);

  std::cout << "Message definition:\n";
  std::cout << "Name: " << msg.GetDescriptor()->full_name() << "\n";
  std::cout << "File: " << msg.GetDescriptor()->file()->name() << "\n\n";
  std::cout << msg.GetDescriptor()->DebugString() << "\n";
  std::cout << "===============================\n";
  std::cout << "Populated message:\n" << msg.DebugString() << std::endl;

  return 0;
}
