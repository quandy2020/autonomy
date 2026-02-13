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
 * Demo: Service client (like the client part of autolink/examples/cpp/service.cpp).
 * Sends Driver requests to "test_server". Run demo_server_reader first.
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

  auto node = autolink::CreateNode("demo_server_writer");
  auto client = node->CreateClient<Driver, Driver>("test_server");
  if (!client) {
    AERROR << "Failed to create service client!";
    return 1;
  }

  AINFO << "Demo service client created. Waiting for test_server...";
  int wait_count = 0;
  while (!client->ServiceIsReady() && wait_count < 100) {
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    wait_count++;
  }
  if (!client->ServiceIsReady()) {
    AERROR << "Service test_server not ready after 10s!";
    return 1;
  }
  AINFO << "Sending Driver requests to test_server at 1 Hz (max 20).";

  Rate rate(1.0);
  int count = 0;
  while (autolink::OK() && count < 20) {
    auto request = std::make_shared<Driver>();
    request->set_msg_id(static_cast<uint64_t>(count));
    request->set_timestamp(0);
    request->set_content("request from demo_server_writer");

    auto response = client->SendRequest(request);
    if (response) {
      AINFO << "Response: msg_id=" << response->msg_id()
            << " content=" << response->content();
    } else {
      AINFO << "No response (service may be busy or unavailable).";
    }
    count++;
    rate.Sleep();
  }

  autolink::WaitForShutdown();
  return 0;
}
