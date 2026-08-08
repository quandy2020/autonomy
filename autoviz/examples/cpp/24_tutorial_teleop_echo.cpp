/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

#include <iostream>
#include <memory>

#include <automsgs/msgs/geometry_msgs/twist.pb.h>
#include <unistd.h>

#include "common/tutorial_utils.hpp"

namespace {

void OnCmd(const std::shared_ptr<automsgs::msgs::geometry_msgs::Twist>& msg) {
  std::cout << "cmd_vel  lin=(" << msg->linear().x() << "," << msg->linear().y()
            << "," << msg->linear().z() << ")  ang=(" << msg->angular().x()
            << "," << msg->angular().y() << "," << msg->angular().z() << ")\n";
}

}  // namespace

int main(int argc, char** argv) {
  if (!autolink::Init(argv[0])) return 1;
  auto node = autolink::CreateNode("/autoviz/teleop_echo");
  auto reader = node->CreateReader<automsgs::msgs::geometry_msgs::Twist>(
      "/cmd_vel", OnCmd);
  if (reader == nullptr) {
    std::cerr << "failed to create reader on /cmd_vel\n";
    return 1;
  }
  std::cout << "listening /cmd_vel — open Autoviz Teleop panel\n";

  while (autolink::OK()) {
    usleep(50000);
  }
  return 0;
}
