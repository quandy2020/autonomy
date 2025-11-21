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

#include "autolink/node/writer.hpp"

#include "gtest/gtest.h"

#include "autolink/proto/unit_test.pb.h"

#include "autolink/autolink.hpp"
#include "autolink/common/init.hpp"

namespace autolink {
namespace writer {

using proto::Chatter;

TEST(WriterTest, test1) {
    proto::RoleAttributes role;
    Writer<Chatter> w(role);
    EXPECT_TRUE(w.Init());
    EXPECT_TRUE(w.IsInit());
    EXPECT_TRUE(w.GetChannelName().empty());
    EXPECT_FALSE(w.HasReader());
}

TEST(WriterTest, test2) {
    proto::RoleAttributes role;
    auto qos = role.mutable_qos_profile();
    qos->set_history(proto::QosHistoryPolicy::HISTORY_KEEP_LAST);
    qos->set_depth(0);
    qos->set_mps(0);
    qos->set_reliability(proto::QosReliabilityPolicy::RELIABILITY_RELIABLE);
    qos->set_durability(proto::QosDurabilityPolicy::DURABILITY_VOLATILE);
    role.set_channel_name("/chatter0");
    role.set_node_name("chatter_node");

    Writer<Chatter> w(role);
    EXPECT_TRUE(w.Init());
    EXPECT_EQ(w.GetChannelName(), "/chatter0");

    {
        auto c = std::make_shared<Chatter>();
        c->set_timestamp(Time::Now().ToNanosecond());
        c->set_lidar_timestamp(Time::Now().ToNanosecond());
        c->set_seq(3);
        c->set_content("ChatterMsg");
        EXPECT_TRUE(w.Write(c));
    }
    EXPECT_TRUE(w.Init());

    w.Shutdown();

    auto c = std::make_shared<Chatter>();
    c->set_timestamp(Time::Now().ToNanosecond());
    c->set_lidar_timestamp(Time::Now().ToNanosecond());
    c->set_seq(3);
    c->set_content("ChatterMsg");
    EXPECT_FALSE(w.Write(c));
}

}  // namespace writer
}  // namespace autolink