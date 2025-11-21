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

#include "autolink/node/node_channel_impl.hpp"

#include <iostream>

#include "gtest/gtest.h"

#include "autolink/proto/unit_test.pb.h"

#include "autolink/autolink.hpp"
#include "autolink/common/init.hpp"
#include "autolink/node/node.hpp"

namespace autolink {
namespace node_channel_impl {

TEST(Node_Channel_ImplTest, test1) {
    auto globalData = common::GlobalData::Instance();
    globalData->DisableSimulationMode();
    {
        NodeChannelImpl nImpl("TestConstructor");
        EXPECT_EQ(nImpl.NodeName(), "TestConstructor");
    }
    globalData->EnableSimulationMode();
    {
        NodeChannelImpl nImpl("TestConstructor");
        EXPECT_EQ(nImpl.NodeName(), "TestConstructor");
    }
}

TEST(Node_Channel_ImplTest, test2) {
    auto globalData = common::GlobalData::Instance();
    std::string nodeName("TestConstructor");
    std::string nameSpace("");
    std::string channelName("/chatter1");
    {
        globalData->DisableSimulationMode();
        auto n = CreateNode(nodeName, nameSpace);
        EXPECT_EQ(n->Name(), nodeName);

        proto::RoleAttributes role_attr;
        auto w = n->CreateWriter<proto::Chatter>(role_attr);
        EXPECT_EQ(w, nullptr);

        role_attr.set_channel_name(std::string());
        w = n->CreateWriter<proto::Chatter>(role_attr);
        EXPECT_EQ(w, nullptr);

        role_attr.set_channel_name(channelName);
        w = n->CreateWriter<proto::Chatter>(role_attr);
        EXPECT_NE(w, nullptr);
    }

    {
        globalData->EnableSimulationMode();
        auto n = CreateNode(nodeName, nameSpace);
        EXPECT_EQ(n->Name(), nodeName);

        proto::RoleAttributes role_attr;
        auto w = n->CreateWriter<proto::Chatter>(role_attr);
        EXPECT_EQ(w, nullptr);

        role_attr.set_channel_name(std::string());
        w = n->CreateWriter<proto::Chatter>(role_attr);
        EXPECT_EQ(w, nullptr);

        role_attr.set_channel_name(channelName);
        w = n->CreateWriter<proto::Chatter>(role_attr);
        EXPECT_NE(w, nullptr);

        w = n->CreateWriter<proto::Chatter>(channelName);
        EXPECT_NE(w, nullptr);
    }
}

TEST(Node_Channel_ImplTest, test3) {
    auto globalData = common::GlobalData::Instance();
    std::string nodeName("TestConstructor");
    std::string nameSpace("");

    auto callback = [](const std::shared_ptr<proto::Chatter>& msg) {
        std::cout << "msg size = " << msg->ByteSizeLong() << std::endl;
    };

    {
        globalData->DisableSimulationMode();
        auto n = CreateNode(nodeName, nameSpace);
        EXPECT_EQ(n->Name(), nodeName);

        n->Observe();
        n->ClearData();

        auto r = n->CreateReader<proto::Chatter>("/chatter1", callback);
        EXPECT_NE(r, nullptr);

        proto::RoleAttributes role_attr;
        r = n->CreateReader<proto::Chatter>(role_attr, callback);
        EXPECT_EQ(r, nullptr);

        role_attr.set_channel_name(std::string());
        r = n->CreateReader<proto::Chatter>(role_attr, callback);
        EXPECT_EQ(r, nullptr);

        role_attr.set_channel_name("/chatter2");
        r = n->CreateReader<proto::Chatter>(role_attr, callback);
        EXPECT_NE(r, nullptr);

        r = n->CreateReader<proto::Chatter>(role_attr);
        EXPECT_EQ(r, nullptr);

        ReaderConfig rc;
        rc.channel_name.assign("/chatter3");
        r = n->CreateReader<proto::Chatter>(rc, callback);
        EXPECT_NE(r, nullptr);

        n->Observe();
        n->ClearData();
    }

    {
        globalData->EnableSimulationMode();
        auto n = CreateNode(nodeName, nameSpace);
        EXPECT_EQ(n->Name(), nodeName);

        n->Observe();
        n->ClearData();

        auto r = n->CreateReader<proto::Chatter>("/chatter1", callback);
        EXPECT_NE(r, nullptr);

        proto::RoleAttributes role_attr;
        r = n->CreateReader<proto::Chatter>(role_attr, callback);
        EXPECT_EQ(r, nullptr);

        role_attr.set_channel_name(std::string());
        r = n->CreateReader<proto::Chatter>(role_attr, callback);
        EXPECT_EQ(r, nullptr);

        role_attr.set_channel_name("/chatter2");
        r = n->CreateReader<proto::Chatter>(role_attr, callback);
        EXPECT_NE(r, nullptr);

        r = n->CreateReader<proto::Chatter>(role_attr);
        EXPECT_EQ(r, nullptr);

        ReaderConfig rc;
        rc.channel_name.assign("/chatter3");
        r = n->CreateReader<proto::Chatter>(rc, callback);
        EXPECT_NE(r, nullptr);

        n->Observe();
        n->ClearData();
    }
}

}  // namespace node_channel_impl
}  // namespace autolink

int main(int argc, char** argv) {
    testing::InitGoogleTest(&argc, argv);
    autolink::Init(argv[0]);
    return RUN_ALL_TESTS();
}