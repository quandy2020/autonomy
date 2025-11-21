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

#include "autolink/autolink.hpp"
#include "autolink/common/init.hpp"
#include "autolink/common/log.hpp"
#include "autolink/time/rate.hpp"
#include "autolink/time/time.hpp"

using autolink::Rate;
using autolink::Time;
using autolink::examples::proto::Chatter;

int received_count = 0;

void MessageCallback(const std::shared_ptr<Chatter>& msg) {
    AINFO << "Received message seq: " << msg->seq();
    AINFO << "Message content: " << msg->content();
    received_count++;
}

int main(int argc, char* argv[]) {
    // init autolink framework
    autolink::Init(argv[0]);
    AINFO << "autolink initialized";

    // Create a single node for both writer and reader
    auto node = autolink::CreateNode("talker_reader");
    AINFO << "node created";

    // Create writer
    auto talker = node->CreateWriter<Chatter>("channel/chatter");
    if (!talker) {
        AERROR << "Failed to create writer!";
        return 1;
    }
    AINFO << "writer created successfully";

    // Create reader with callback
    auto listener =
        node->CreateReader<Chatter>("channel/chatter", MessageCallback);
    if (!listener) {
        AERROR << "Failed to create reader!";
        return 1;
    }
    AINFO << "reader created successfully";

    Rate rate(1.0);
    uint64_t seq = 0;

    AINFO << "Starting message loop";

    for (int i = 0; i < 10; i++) {
        auto msg = std::make_shared<Chatter>();
        msg->set_timestamp(Time::Now().ToNanosecond());
        msg->set_lidar_timestamp(Time::Now().ToNanosecond());
        msg->set_seq(seq);
        msg->set_content("Hello, autolink!");

        talker->Write(msg);
        AINFO << "talker sent a message! No. " << seq;

        seq++;
        rate.Sleep();
    }

    AINFO << "Exiting message loop";
    AINFO << "Total received: " << received_count << " messages";

    autolink::WaitForShutdown();
    return 0;
}
