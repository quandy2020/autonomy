/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

#include <chrono>
#include <iostream>
#include <memory>
#include <thread>
#include <vector>

#include "autolink/parameter/parameter.hpp"
#include "autolink/parameter/parameter_server.hpp"

#include "common/tutorial_utils.hpp"

int main(int argc, char** argv) {
  (void)argc;
  if (!autolink::Init(argv[0])) return 1;

  auto node = autolink::CreateNode("/autoviz/params");
  auto srv = std::make_shared<autolink::ParameterServer>(node);
  std::this_thread::sleep_for(std::chrono::milliseconds(300));

  srv->SetParameter(autolink::Parameter("max_speed", 1.5));
  srv->SetParameter(autolink::Parameter("goal_tol", 0.05));
  srv->SetParameter(autolink::Parameter("enable_estop", false));
  srv->SetParameter(autolink::Parameter("waypoint_count", 8));
  srv->SetParameter(autolink::Parameter("robot_name", std::string("demo_bot")));

  std::vector<autolink::Parameter> listed;
  srv->ListParameters(&listed);
  std::cout << "ParameterServer /autoviz/params (" << listed.size()
            << " params):\n";
  for (const auto& p : listed) {
    std::cout << "  " << p.Name() << ": " << p.DebugString() << "\n";
  }
  std::cout << "Open Autoviz Parameters → select this node → edit Value\n";

  while (autolink::OK()) {
    std::this_thread::sleep_for(std::chrono::milliseconds(200));
  }
  return 0;
}
