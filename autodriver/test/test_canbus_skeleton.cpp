/*
 * Copyright 2026 Autodriver contributors
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

#include <gtest/gtest.h>

#include "autodriver/canbus/byte.hpp"
#include "autodriver/canbus/can_client.hpp"
#include "autodriver/canbus/protocol_data.hpp"

TEST(CanbusByte, SetGetBits) {
    std::uint8_t raw = 0;
    autodriver::canbus::Byte b(&raw);
    b.set_value(0xA, 0, 4);
    EXPECT_EQ(b.get_byte(0, 4), 0xA);
    b.set_bit(7);
    EXPECT_TRUE(b.is_bit_1(7));
    EXPECT_EQ(autodriver::canbus::Byte::byte_to_hex(0x1F), "1F");
}

TEST(FakeCanClient, SendReceive) {
    autodriver::canbus::FakeCanClient::Clear("fake0");
    auto client = autodriver::canbus::CreateCanClient("fake0");
    ASSERT_NE(client, nullptr);
    ASSERT_TRUE(client->Init("fake0"));
    autodriver::io::CanFrame tx{};
    tx.id = 0x123;
    tx.dlc = 2;
    tx.data[0] = 0xAB;
    tx.data[1] = 0xCD;
    EXPECT_TRUE(client->Send(tx));
    autodriver::io::CanFrame rx{};
    EXPECT_TRUE(client->Receive(&rx, 0));
    EXPECT_EQ(rx.id, 0x123u);
    EXPECT_EQ(rx.data[0], 0xAB);
    EXPECT_EQ(rx.data[1], 0xCD);
}

TEST(MessageManager, StillDispatches) {
    struct Msg {
        int v = 0;
    };
    class Proto : public autodriver::canbus::ProtocolData<Msg> {
    public:
        std::uint32_t can_id() const override { return 0x42; }
        bool Parse(const autodriver::io::CanFrame& frame,
                   Msg* msg) const override {
            msg->v = frame.data[0];
            return true;
        }
    };
    autodriver::canbus::MessageManager<Msg> manager;
    manager.Register(std::make_shared<Proto>());
    int got = -1;
    manager.SetPublishCallback([&](const Msg& m) { got = m.v; });
    autodriver::io::CanFrame frame{};
    frame.id = 0x42;
    frame.dlc = 1;
    frame.data[0] = 9;
    EXPECT_TRUE(manager.Parse(frame));
    EXPECT_EQ(got, 9);
}
