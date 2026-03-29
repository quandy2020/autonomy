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
#include "autolink/time/rate.hpp"
#include "autolink/time/time.hpp"
#include "examples.pb.h"

using autolink::Rate;
using autolink::Time;
using autolink::examples::Chatter;

void MessageCallback(const std::shared_ptr<Chatter>& msg) {
    AINFO << "Received message seq-> " << msg->seq() << " content-> "
          << msg->content();
}

int main(int argc, char* argv[]) {
    if (!autolink::Init(argv[0]))
        return 1;

    auto node = autolink::CreateNode("talker_listener");
    auto writer = node->CreateWriter<Chatter>("channel/chatter");
    auto reader =
        node->CreateReader<Chatter>("channel/chatter", MessageCallback);

    Rate rate(1.0);
    uint64_t seq = 0;
    while (autolink::OK()) {
        auto msg = std::make_shared<Chatter>();
        msg->set_timestamp(Time::Now().ToNanosecond());
        msg->set_lidar_timestamp(Time::Now().ToNanosecond());
        msg->set_seq(seq);
        msg->set_content("Hello, autolink!");
        writer->Write(msg);
        AINFO << "sent seq " << seq;
        seq++;
        rate.Sleep();
    }
    return 0;
}
