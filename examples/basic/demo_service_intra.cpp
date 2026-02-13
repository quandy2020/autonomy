/**
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
 *
 * Demo: Intra-process Service (server + client in same process).
 * Runs both Driver service server and client in one process on test_server,
 * demonstrating intra-process service communication without network.
 */

#include "proto/examples.pb.h"

#include <chrono>
#include <thread>

#include "autolink/autolink.hpp"
#include "autolink/common/init.hpp"
#include "autolink/common/log.hpp"
#include "autolink/common/state.hpp"
#include "autolink/time/rate.hpp"

using autolink::examples::proto::Driver;
using autolink::Rate;

int main(int argc, char* argv[]) {
  autolink::Init(argv[0]);

  auto node = autolink::CreateNode("demo_service_intra");

  // Create service server first
  auto server = node->CreateService<Driver, Driver>(
      "test_server",
      [](const std::shared_ptr<Driver>& request,
         std::shared_ptr<Driver>& response) {
        AINFO << "[Server] Received request msg_id=" << request->msg_id();
        static uint64_t id = 0;
        ++id;
        response->set_msg_id(id);
        response->set_timestamp(request->timestamp() + 1);
        response->set_content("response from intra-process demo");
      });

  // Create service client
  auto client = node->CreateClient<Driver, Driver>("test_server");

  // Intra-process: server is in same process, brief wait for registration
  int wait_count = 0;
  while (!client->ServiceIsReady() && wait_count < 100) {
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
    wait_count++;
  }
  if (!client->ServiceIsReady()) {
    AERROR << "Service test_server not ready!";
    return 1;
  }

  Rate rate(1.0);
  int count = 0;
  while (autolink::OK() && count < 10) {
    auto request = std::make_shared<Driver>();
    request->set_msg_id(static_cast<uint64_t>(count));
    request->set_timestamp(0);
    request->set_content("request from intra-process demo");

    auto response = client->SendRequest(request);
    if (response) {
      AINFO << "[Client] Response: msg_id=" << response->msg_id()
            << " content=" << response->content();
    } else {
      AINFO << "[Client] No response.";
    }
    count++;
    rate.Sleep();
  }

  AINFO << "Demo completed (intra-process).";
  autolink::WaitForShutdown();
  return 0;
}
