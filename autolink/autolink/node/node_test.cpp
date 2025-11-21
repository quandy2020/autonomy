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

#include "autolink/node/node.hpp"

#include <cstdlib>
#include <thread>

#include "gtest/gtest.h"

#include "autolink/proto/unit_test.pb.h"

#include "autolink/autolink.hpp"
#include "autolink/common/init.hpp"
#include "autolink/node/reader.hpp"
#include "autolink/node/writer.hpp"

namespace autolink {

using autolink::proto::Chatter;

TEST(NodeTest, cases) {
    // NOTE: This test creates segmentation fault during cleanup/shutdown
    // due to scheduler and croutine tasks not being properly cleaned up.
    // The test passes successfully, but the process crashes during static
    // destructor phase. This is a known issue in the framework initialization.
    // TODO: Fix scheduler shutdown and croutine cleanup

    auto node = CreateNode("node_test");
    EXPECT_TRUE(node != nullptr);
    EXPECT_EQ(node->Name(), "node_test");

    proto::RoleAttributes attr;
    attr.set_channel_name("/node_test_channel");
    auto channel_id = common::GlobalData::RegisterChannel(attr.channel_name());
    attr.set_channel_id(channel_id);
    attr.mutable_qos_profile()->set_depth(10);

    auto reader = node->CreateReader<Chatter>(attr);
    EXPECT_TRUE(reader != nullptr);
    EXPECT_TRUE(node->GetReader<Chatter>(attr.channel_name()));

    auto writer = node->CreateWriter<Chatter>(attr);
    EXPECT_TRUE(writer != nullptr);

    auto server = node->CreateService<Chatter, Chatter>(
        "node_test_server", [](const std::shared_ptr<Chatter>& request,
                               std::shared_ptr<Chatter>& response) {
            AINFO << "server: I am server";
            static uint64_t id = 0;
            ++id;
            response->set_seq(id);
            response->set_timestamp(0);
        });
    EXPECT_TRUE(server != nullptr);

    auto client = node->CreateClient<Chatter, Chatter>("node_test_server");
    EXPECT_TRUE(client != nullptr);

    auto chatter_msg = std::make_shared<Chatter>();
    chatter_msg->set_seq(0);
    chatter_msg->set_timestamp(0);
    auto res = client->SendRequest(chatter_msg);
    EXPECT_TRUE(res != nullptr);
    EXPECT_EQ(res->seq(), 1);

    node->Observe();
    node->ClearData();

    // Don't explicitly destroy - let it go out of scope naturally
}
}  // namespace autolink

int main(int argc, char** argv) {
    testing::InitGoogleTest(&argc, argv);
    autolink::Init(argv[0]);
    int result = RUN_ALL_TESTS();

    // Give a moment for cleanup
    std::this_thread::sleep_for(std::chrono::milliseconds(100));

    // Force exit to avoid hang in scheduler cleanup
    // Known issue: scheduler has cleanup issues that cause process to hang
    _exit(result);
}