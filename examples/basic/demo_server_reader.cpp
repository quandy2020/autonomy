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
 * Demo: Service server (like the server part of autolink/examples/cpp/service.cpp).
 * Provides "test_server" with Driver request/response. Use demo_server_writer
 * as client to send requests.
 */

#include "proto/examples.pb.h"

#include "autolink/autolink.hpp"
#include "autolink/common/init.hpp"
#include "autolink/common/log.hpp"
#include "autolink/common/state.hpp"

using autolink::examples::proto::Driver;

int main(int argc, char* argv[]) {
  autolink::Init(argv[0]);

  auto node = autolink::CreateNode("demo_server_reader");
  auto server = node->CreateService<Driver, Driver>(
      "test_server",
      [](const std::shared_ptr<Driver>& request,
         std::shared_ptr<Driver>& response) {
        AINFO << "Server received request msg_id=" << request->msg_id();
        static uint64_t id = 0;
        ++id;
        response->set_msg_id(id);
        response->set_timestamp(
            request->timestamp() + 1);
        response->set_content("response from demo_server_reader");
      });

  if (!server) {
    AERROR << "Failed to create service server!";
    return 1;
  }

  AINFO << "Demo service server ready on test_server. Waiting for requests...";
  autolink::WaitForShutdown();
  return 0;
}
