/******************************************************************************
 * Copyright 2018 The Autolink Authors. All Rights Reserved.
 *****************************************************************************/

#include "autolink/autolink.hpp"
#include "autolink/time/rate.hpp"
#include "autolink/time/time.hpp"
#include "examples.pb.h"

using autolink::Rate;
using autolink::Time;
using autolink::examples::Chatter;

int main(int argc, char* argv[]) {
    if (!autolink::Init(argv[0]))
        return 1;
    auto talker_node = autolink::CreateNode("talker");
    auto talker = talker_node->CreateWriter<Chatter>("channel/chatter");
    Rate rate(1.0);
    uint64_t seq = 0;
    while (autolink::OK()) {
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
    return 0;
}
