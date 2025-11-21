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

#include <memory>
#include <string>

#include "autolink/autolink.hpp"
#include "autolink/common/init.hpp"
#include "autolink/common/log.hpp"
#include "autolink/common/state.hpp"

using autolink::examples::proto::Chatter;

void MessageCallback(
    const std::shared_ptr<autolink::examples::proto::Chatter>& msg) {
  AINFO << "Received message seq-> " << msg->seq();
  AINFO << "msgcontent->" << msg->content();
}

int main(int argc, char* argv[]) {
  // init cyber framework
  autolink::Init(argv[0]);
  // create listener node
  auto listener_node = autolink::CreateNode("listener");
  // create listener
  auto listener = listener_node->CreateReader<autolink::examples::proto::Chatter>(
          "channel/chatter", MessageCallback);
  if (!listener) {
    AERROR << "Failed to create reader!";
    return 1;
  }
  AINFO << "Reader created, waiting for messages...";
  autolink::WaitForShutdown();
  return 0;
}