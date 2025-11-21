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
 */

#include "autolink/examples/proto/examples.pb.h"

#include <unistd.h>
#include <atomic>
#include <chrono>
#include <iostream>
#include <thread>

#include "autolink/autolink.hpp"
#include "autolink/common/init.hpp"
#include "autolink/common/log.hpp"
#include "autolink/common/state.hpp"
#include "autolink/time/rate.hpp"
#include "autolink/time/time.hpp"

using autolink::examples::proto::Driver;
using autolink::Rate;
using autolink::Time;

int main(int argc, char *argv[]) {
  // init autolink framework
  autolink::Init(argv[0]);
  // create writer node
  auto writer_node = autolink::CreateNode("channel_test_writer");
  // create writer
  auto writer = writer_node->CreateWriter<Driver>("/apollo/test");
  if (!writer) {
    AERROR << "Failed to create writer!";
    return 1;
  }
  AINFO << "Writer created, waiting for topology discovery...";
  // Wait a bit for topology discovery to complete
  std::this_thread::sleep_for(std::chrono::milliseconds(500));

  Rate rate(2.0);
  std::string content("apollo_test");
  while (autolink::OK()) {
    static uint64_t seq = 0;
    auto msg = std::make_shared<Driver>();
    msg->set_timestamp(Time::Now().ToNanosecond());
    msg->set_msg_id(seq++);
    msg->set_content(content + std::to_string(seq - 1));
    if (writer->Write(msg)) {
      AINFO << "/apollo/test sent message, seq=" << (seq - 1) << ";";
    } else {
      AWARN << "Failed to send message, seq=" << (seq - 1) << ";";
    }
    rate.Sleep();
  }
  autolink::WaitForShutdown();
  return 0;
}
