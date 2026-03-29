/******************************************************************************
 * Copyright 2025 The Openbot Authors (duyongquan)
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *****************************************************************************/

#include "autolink/autolink.hpp"
#include "examples.pb.h"

void MessageCallback(const std::shared_ptr<autolink::examples::Chatter>& msg) {
    AINFO << "Received message seq-> " << msg->seq();
    AINFO << "msgcontent->" << msg->content();
}

int main(int argc, char* argv[]) {
    if (!autolink::Init(argv[0]))
        return 1;
    auto listener_node = autolink::CreateNode("listener");
    auto listener = listener_node->CreateReader<autolink::examples::Chatter>(
        "channel/chatter", MessageCallback);
    AINFO << "listener subscribed to channel/chatter, waiting for messages";
    AINFO << "  (run autolink_example_talker in another terminal; "
             "if no 'SHM receiver enabled for channel' in log, see "
             "examples/TROUBLESHOOT_LISTENER.md)";
    autolink::WaitForShutdown();
    return 0;
}
