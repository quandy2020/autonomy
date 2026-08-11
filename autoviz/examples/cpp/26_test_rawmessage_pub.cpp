/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

#include <chrono>
#include <iostream>
#include <memory>
#include <thread>

#include <automsgs/msgs/geometry_msgs/twist.pb.h>
#include <unistd.h>

#include "autolink/autolink.hpp"
#include "autolink/init.hpp"
#include "autolink/message/raw_message.hpp"
#include "autolink/service_discovery/topology_manager.hpp"

int main(int argc, char** argv) {
  if (!autolink::Init(argv[0])) {
    return 1;
  }
  auto node = autolink::CreateNode("rawmessage_pub_test");
  auto writer =
      node->CreateWriter<autolink::message::RawMessage>("/cmd_vel");
  if (writer == nullptr) {
    std::cerr << "failed to create writer\n";
    return 1;
  }

  automsgs::msgs::geometry_msgs::Twist twist;
  twist.mutable_linear()->set_x(3.5);
  std::string payload;
  twist.SerializeToString(&payload);
  auto msg = std::make_shared<autolink::message::RawMessage>();
  msg->message = payload;

  auto* topology = autolink::service_discovery::TopologyManager::Instance();
  for (int i = 0; i < 80; ++i) {
    const bool topo_reader =
        topology->channel_manager()->HasReader("/cmd_vel");
    const bool writer_reader = writer->HasReader();
    const bool ok = writer->Write(msg);
    std::cout << "i=" << i << " write=" << ok
              << " topo_reader=" << topo_reader
              << " writer_reader=" << writer_reader << std::endl;
    if (ok) {
      return 0;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
  }
  return 1;
}
