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
 * Demo: Intra-process Channel (writer + reader in same process).
 * Runs both Chatter writer and reader in one process on channel/chatter,
 * demonstrating intra-process channel communication without network.
 */

#include "autolink/proto/unit_test.pb.h"

#include <chrono>
#include <thread>

#include "autolink/autolink.hpp"
#include "autolink/common/init.hpp"
#include "autolink/common/log.hpp"
#include "autolink/common/state.hpp"
#include "autolink/time/rate.hpp"
#include "autolink/time/time.hpp"

using autolink::Rate;
using autolink::Time;
using autolink::proto::Chatter;

void MessageCallback(const std::shared_ptr<Chatter>& msg) {
  AINFO << "[Reader] Received seq=" << msg->seq()
        << " content=" << msg->content();
}

int main(int argc, char* argv[]) {
  autolink::Init(argv[0]);

  auto node = autolink::CreateNode("demo_channel_intra");

  // Create reader first, then writer (same channel)
  auto reader = node->CreateReader<Chatter>("channel/chatter", MessageCallback);
  auto writer = node->CreateWriter<Chatter>("channel/chatter");

  Rate rate(1.0);
  uint64_t seq = 0;

  while (autolink::OK()) {
    auto msg = std::make_shared<Chatter>();
    msg->set_timestamp(Time::Now().ToNanosecond());
    msg->set_lidar_timestamp(Time::Now().ToNanosecond());
    msg->set_seq(seq);
    msg->set_content("Hello from intra-process demo!");
    if (writer->Write(msg)) {
      AINFO << "[Writer] Sent message #" << seq;
    }
    seq++;
    rate.Sleep();
  }

  autolink::WaitForShutdown();
  return 0;
}
