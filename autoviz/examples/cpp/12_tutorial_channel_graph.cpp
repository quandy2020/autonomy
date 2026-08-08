/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

#include <chrono>
#include <iostream>
#include <memory>
#include <string>

#include <automsgs/msgs/std_msgs/string.pb.h>

#include "common/tutorial_utils.hpp"

using String = automsgs::msgs::std_msgs::String;
using autoviz::examples::MakeWriter;
using autoviz::examples::ParseRate;

int main(int argc, char** argv) {
  const double rate_hz = ParseRate(argc, argv, 5.0);

  if (!autolink::Init(argv[0])) return 1;

  auto sensor = autolink::CreateNode("/demo/sensor");
  auto perception = autolink::CreateNode("/demo/perception");
  auto planner = autolink::CreateNode("/demo/planner");
  auto controller = autolink::CreateNode("/demo/controller");
  auto map_server = autolink::CreateNode("/demo/map_server");

  auto w_scan = MakeWriter<String>(sensor, "/fake/graph/scan");
  auto w_imu = MakeWriter<String>(sensor, "/fake/graph/imu");
  auto w_obj = MakeWriter<String>(perception, "/fake/graph/objects");
  auto w_path = MakeWriter<String>(planner, "/fake/graph/path");
  auto w_cmd = MakeWriter<String>(controller, "/fake/graph/cmd");

  auto noop = [](const std::shared_ptr<String>&) {};
  perception->CreateReader<String>("/fake/graph/scan", noop);
  perception->CreateReader<String>("/fake/graph/imu", noop);
  planner->CreateReader<String>("/fake/graph/objects", noop);
  controller->CreateReader<String>("/fake/graph/path", noop);

  map_server->CreateService<String, String>(
      "/fake/graph/get_map",
      [](const std::shared_ptr<String>& req,
         std::shared_ptr<String>& resp) {
        resp->set_data("map:" + req->data());
      });
  auto client = planner->CreateClient<String, String>("/fake/graph/get_map");

  autolink::Rate rate(rate_hz);
  std::cout << "channel graph @ " << rate_hz
            << " Hz → /fake/graph/* (+ get_map service)\n";

  int i = 0;
  while (autolink::OK()) {
    auto scan = std::make_shared<String>();
    scan->set_data("scan:" + std::to_string(i));
    w_scan->Write(scan);

    auto imu = std::make_shared<String>();
    imu->set_data("imu:" + std::to_string(i));
    w_imu->Write(imu);

    auto obj = std::make_shared<String>();
    obj->set_data("objects:" + std::to_string(i));
    w_obj->Write(obj);

    auto path = std::make_shared<String>();
    path->set_data("path:" + std::to_string(i));
    w_path->Write(path);

    auto cmd = std::make_shared<String>();
    cmd->set_data("cmd:" + std::to_string(i));
    w_cmd->Write(cmd);

    if (i % 10 == 0 && client) {
      auto req = std::make_shared<String>();
      req->set_data("floor1");
      client->SendRequest(req, std::chrono::seconds(1));
    }
    ++i;
    rate.Sleep();
  }
  return 0;
}
