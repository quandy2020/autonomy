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

using autolink::examples::proto::Chatter;
using autolink::Rate;
using autolink::Time;

int main(int argc, char *argv[]) {
  // init cyber framework
  autolink::Init(argv[0]);
  // create talker node
  auto talker_node = autolink::CreateNode("talker");
  // create talker
  auto talker = talker_node->CreateWriter<Chatter>("channel/chatter");
  if (!talker) {
    AERROR << "Failed to create writer!";
    return 1;
  }
  AINFO << "Writer created, waiting for topology discovery...";
  // Wait a bit for topology discovery to complete
  std::this_thread::sleep_for(std::chrono::milliseconds(500));
  
  // Create rate limiter: 1.0 Hz = 1 message per second
  Rate rate(1.0);  // 每秒发送一条消息
  uint64_t seq = 0;
  AINFO << "Starting to send messages at 1 Hz (1 message per second)";
  
  while (autolink::OK()) {
    auto msg = std::make_shared<Chatter>();
    msg->set_timestamp(Time::Now().ToNanosecond());
    msg->set_lidar_timestamp(Time::Now().ToNanosecond());
    msg->set_seq(seq);
    msg->set_content("Hello, autolink!");
    if (talker->Write(msg)) {
      AINFO << "talker sent a message! No. " << seq;
    } else {
      AWARN << "Failed to send message No. " << seq;
    }
    seq++;
    // Sleep to maintain 1 Hz rate (1 second between messages)
    rate.Sleep();
  }

  autolink::WaitForShutdown();
  return 0;
}
