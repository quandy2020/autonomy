/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

#include <iostream>
#include <memory>
#include <string>

#include <automsgs/msgs/std_msgs/string.pb.h>
#include <unistd.h>

#include "common/tutorial_utils.hpp"

using String = automsgs::msgs::std_msgs::String;

int main(int argc, char** argv) {
  if (!autolink::Init(argv[0])) return 1;
  auto node = autolink::CreateNode("/autoviz/service");
  auto server = node->CreateService<String, String>(
      "/fake/echo",
      [](const std::shared_ptr<String>& req, std::shared_ptr<String>& res) {
        std::cout << "echo ← " << req->data() << "\n";
        res->set_data("echo:" + req->data());
      });
  if (server == nullptr) {
    std::cerr << "failed to create /fake/echo service\n";
    return 1;
  }
  std::cout << "Service /fake/echo ready — use Autoviz Service Call panel\n";

  while (autolink::OK()) {
    usleep(200000);
  }
  return 0;
}
