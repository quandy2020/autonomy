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
 * Demo: Channel reader (like autolink/examples/cpp/listener.cpp).
 * Subscribes to channel/chatter and prints received Chatter messages.
 */

#include "autolink/proto/unit_test.pb.h"

#include <memory>
#include <string>

#include "autolink/autolink.hpp"
#include "autolink/common/init.hpp"
#include "autolink/common/log.hpp"
#include "autolink/common/state.hpp"

using autolink::proto::Chatter;

void MessageCallback(const std::shared_ptr<Chatter>& msg) {
  AINFO << "Received seq=" << msg->seq() << " timestamp=" << msg->timestamp()
        << " content=" << msg->content();
}

int main(int argc, char* argv[]) {
  autolink::Init(argv[0]);

  auto node = autolink::CreateNode("demo_channel_reader");
  auto reader =
      node->CreateReader<Chatter>("channel/chatter", MessageCallback);
  if (!reader) {
    AERROR << "Failed to create reader!";
    return 1;
  }
  AINFO << "Demo channel reader created, waiting for messages on channel/chatter...";
  autolink::WaitForShutdown();
  return 0;
}
